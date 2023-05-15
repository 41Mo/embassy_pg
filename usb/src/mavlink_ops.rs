use defmt::{debug, trace};
use embassy_sync::blocking_mutex::raw as raw_blocking_mutex;
use embassy_sync::signal;
use embassy_time::{Duration, Timer};
use futures::pin_mut;
use mavlink;

// #[derive(defmt::Format)]
// struct HdrWrapper(/* #[defmt(Debug2Format)] */ mavlink::MavHeader);

// #[derive(defmt::Format)]
// struct ErrMsgWrapper(/* #[defmt(Debug2Format)] */ mavlink::error::MessageReadError);

// #[derive(defmt::Format)]
// struct MavMsgWrapper(/* #[defmt(Debug2Format)] */ mavlink::common::MavMessage);

static MSG_NEXT: signal::Signal<raw_blocking_mutex::CriticalSectionRawMutex, Msgs> =
    signal::Signal::new();

fn move_packet_in_buff(buf: &mut [u8], tail_pos: &mut usize, initial_offset: usize) -> bool {
    let mut header_offset = 0;
    let mut packet_found = false;
    for i in initial_offset..*tail_pos {
        header_offset = i;
        if buf[i] == mavlink::MAV_STX {
            packet_found = true;
            break;
        }
    }

    if header_offset != 0 {
        buf.copy_within(header_offset..*tail_pos, 0);
        *tail_pos -= header_offset;
    }

    return packet_found;
}

async fn parse_message<R: crate::traits::Reader>(
    rx: &mut R,
    buf: &mut [u8],
    tail_pos: &mut usize,
) -> Option<mavlink::MAVLinkV1MessageRaw> {
    match rx.read(&mut buf[*tail_pos..]).await {
        Ok(v) => *tail_pos += v,
        Err(e) => {
            trace!("rx err {}", e);
            return None;
        }
    }

    if !move_packet_in_buff(buf, tail_pos, 0) {
        return None;
    }

    let mut message = mavlink::MAVLinkV1MessageRaw::new();
    let len = message.0.len();
    message.0.clone_from_slice(&buf[..len]);
    if !message.has_valid_crc::<mavlink::common::MavMessage>() {
        // bad crc: ignore message
        debug!(
            "crc: {}, should be {}",
            message.checksum(),
            message.calculate_crc::<mavlink::common::MavMessage>()
        );
        move_packet_in_buff(buf, tail_pos, 1);
        return None;
    }
    *tail_pos = 0;
    return Some(message);
}

async fn update_receive<R: crate::traits::Reader>(rx: &mut R) {
    let mut buf = [0u8; 600];
    let mut tail_pos = 0;
    let mut connected = false;
    loop {
        let r = embassy_time::with_timeout(
            embassy_time::Duration::from_secs(5),
            parse_message(rx, &mut buf, &mut tail_pos),
        )
        .await
        .ok();
        if r.is_none() {
            return;
        }

        let mut message = match r.unwrap() {
            Some(m) => m,
            None => continue,
        };

        let msg = match mavlink::deserialize_v1_msg::<mavlink::common::MavMessage>(&mut message) {
            Ok(v) => v,
            Err(_) => {
                debug!("deserialize_v1_msg err");
                continue;
            }
        };

        match msg.1 {
            mavlink::common::MavMessage::HEARTBEAT(hb) => {
                if hb.mavtype == mavlink::common::MavType::MAV_TYPE_GCS && !connected {
                    connected = true;
                    debug!(
                        "Connected to: sysid {}. compid {}",
                        msg.0.system_id, msg.0.component_id
                    )
                }
            }
            _ => ()//debug!("msg: {}", MavMsgWrapper(msg.1)),
        }
    }
}

fn heartbeat_msg() -> mavlink::common::MavMessage {
    mavlink::common::MavMessage::HEARTBEAT(mavlink::common::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: mavlink::common::MavType::MAV_TYPE_FIXED_WING,
        autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode: mavlink::common::MavModeFlag::empty(),
        system_status: mavlink::common::MavState::MAV_STATE_ACTIVE,
        mavlink_version: 0x3,
    })
}

fn scaled_imu() -> mavlink::common::MavMessage {
    mavlink::common::MavMessage::SCALED_IMU(mavlink::common::SCALED_IMU_DATA {
        time_boot_ms: embassy_time::Instant::now().as_millis() as u32,
        xacc: 0,
        yacc: 0,
        zacc: 9810,
        xgyro: 0,
        ygyro: 0,
        zgyro: 0,
        xmag: 0,
        ymag: 0,
        zmag: 0,
    })
}

enum Msgs {
    HEARTBEAT,
    IMU,
}

async fn schedule_msg() {
    let hb_fut = async {
        loop {
            Timer::after(Duration::from_hz(1)).await;
            MSG_NEXT.signal(Msgs::HEARTBEAT)
        }
    };

    let scaled_imu_fut = async {
        loop {
            Timer::after(Duration::from_hz(200)).await;
            MSG_NEXT.signal(Msgs::IMU)
        }
    };

    pin_mut!(scaled_imu_fut, hb_fut);

    embassy_futures::join::join(scaled_imu_fut, hb_fut).await;
}

async fn update_send<W: crate::traits::Writer>(tx: &mut W) {
    let mut mav_header: mavlink::MavHeader = mavlink::MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 0,
    };
    let mut msg = mavlink::MAVLinkV1MessageRaw::new();
    loop {
        let next_msg = MSG_NEXT.wait().await;
        mav_header.sequence = mav_header.sequence.overflowing_add(1).0;
        match next_msg {
            Msgs::HEARTBEAT => msg.serialize_message(mav_header, &heartbeat_msg()),
            Msgs::IMU => msg.serialize_message(mav_header, &scaled_imu()),
        }

        match tx.write(msg.as_bytes()).await {
            Ok(_) => trace!("send ok"),
            Err(e) => trace!("send err: {}", e),
        }
    }
}

#[embassy_executor::task]
pub async fn mavlnk_task(
    mut usb: crate::usb::USB,
    mut rx: crate::usb::Reciever,
    mut tx: crate::usb::Sender,
) {
    loop {
        {
            let rx_fut = update_receive(&mut rx);
            let tx_fut = update_send(&mut tx);
            let msg_sched_fut = schedule_msg();
            let usb_run_fut = usb.run();
            pin_mut!(rx_fut, tx_fut, msg_sched_fut, usb_run_fut);
            embassy_futures::select::select4(rx_fut, tx_fut, msg_sched_fut, usb_run_fut).await;
        }

        usb.0.disable().await;
        debug!("restart mavlink task");
    }
}
