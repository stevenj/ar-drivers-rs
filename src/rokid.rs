// Copyright (C) 2023, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

//! Rokid Air AR glasses support. See [`RokidAir`]
//! It only uses [`rusb`] for communication.
//! 


// # Rokid Commands
//
//         Request, Index,   Value, Data
// ## Display Mode
//
// * Get -    0x81,  0x01,    0x00, [Mode : u8]  (Value could also be 1 when ?? == (0x162f)??)
// * Set -    0x01,  0x01, mode:u8, [0x01: u8]
//
// ## Volume
//
// * Get -    0x81,  0x0a,    0x00, [Volume : u8: 0x40]
// * Set -    0x01,  0x0a,  vol:u8, [0x01: u8]
//
// ## Brightness
//
// * Get -    0x82,  0x02,    0x00, [Brightness : u8]
// * Set -    0x02,  0x02, (b1 | b2) | u16, [0x01: u8]
//
// ## HArdware Stats
//
// * FW Version - 0x81, 0x00, 0x00, [0x40 bytes]
// * HW Version - 0x81, 0x00, 0x800, [0x10 bytes]
// * Optical ID - 0x81, 0x00, 0x700, [0x40 bytes]
// * PCBA       - 0x81, 0x00, 0x200, [0x40 bytes]
// * Seed       - 0x81, 0x00, 0xa00, [0x40 bytes]
// * Serial#    - 0x81, 0x00, 0x100, [0x40 bytes]
// * TypeID     - 0x81, 0x00, 0x300, [0x40 bytes]
// 
// ## Interesting Commands
//
// * Unlock - 0x01, 0x02, 0x400, "E22F1731F48B45E99845ECB28192A17D"+0x00 (0x21 bytes)
//
// * Get Keymask Node - 0x81, 0x00, 0x3200, [0x1 bytes]
// * Set Keymask Node - 0x01, 0x00, 0x3200, [0x1 bytes]

use std::{collections::VecDeque, time::Duration};

use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use rusb::{request_type, DeviceHandle, GlobalContext};

use crate::{
    util::get_interface_for_endpoint, ARGlasses, DisplayMode, Error, GlassesEvent, Result, Side,
};

/// The main structure representing a connected Rokid Air glasses
pub struct RokidAir {
    device_handle: DeviceHandle<GlobalContext>,
    last_accelerometer: Option<(Vector3<f32>, u64)>,
    last_gyroscope: Option<(Vector3<f32>, u64)>,
    previous_key_states: u8,
    proxy_sensor_was_far: bool,
    pending_events: VecDeque<GlassesEvent>,
    model: RokidModel,
}

enum RokidModel {
    Air,
    Max,
}

/* This is actually hardcoded in the SDK too, except for PID==0x162d, where it's 0x83 */
const INTERRUPT_IN_ENDPOINT: u8 = 0x82;

const TIMEOUT: Duration = Duration::from_millis(250);

impl ARGlasses for RokidAir {
    fn serial(&mut self) -> Result<String> {
        let mut result = [0u8; 0x40];
        self.device_handle.read_control(
            request_type(
                rusb::Direction::In,
                rusb::RequestType::Vendor,
                rusb::Recipient::Device,
            ),
            0x81,
            0x100,
            0,
            &mut result,
            TIMEOUT,
        )?;
        Ok(
            String::from_utf8(result.iter().copied().take_while(|c| *c != 0).collect())
                .map_err(|_| "Invalid serial string")?,
        )
    }

    fn read_event(&mut self) -> Result<GlassesEvent> {
        while self.pending_events.is_empty() {
            let mut packet_data = [0u8; 0x40];
            self.device_handle
                .read_interrupt(INTERRUPT_IN_ENDPOINT, &mut packet_data, TIMEOUT)?;
            match packet_data[0] {
                2 => {
                    let packet: &MiscPacket = bytemuck::cast_ref(&packet_data);
                    self.handle_key_press(packet.keys_pressed);
                    self.handle_proxy_sensor(packet.proxy_sensor);
                }
                4 => {
                    let packet: &SensorPacket = bytemuck::cast_ref(&packet_data);
                    let sensor_data =
                        Vector3::from_data(nalgebra::ArrayStorage([packet.vector; 1]));
                    match packet.sensor_type {
                        1 => self.last_accelerometer = Some((sensor_data, packet.timestamp)),
                        2 => self.last_gyroscope = Some((sensor_data, packet.timestamp)),
                        // TODO: Magnetometer apparently gives an accuracy value too
                        3 => self.pending_events.push_back(GlassesEvent::Magnetometer {
                            magnetometer: sensor_data,
                            timestamp: packet.timestamp,
                        }),
                        _ => (),
                    }
                    if let (Some((accelerometer, acc_ts)), Some((gyroscope, gyro_ts))) =
                        (self.last_accelerometer, self.last_gyroscope)
                    {
                        if acc_ts == gyro_ts {
                            self.last_gyroscope = None;
                            self.last_accelerometer = None;
                            self.pending_events.push_back(GlassesEvent::AccGyro {
                                accelerometer,
                                gyroscope,
                                timestamp: acc_ts,
                            });
                        }
                    }
                }
                17 => {
                    let packet: &CombinedPacket = bytemuck::cast_ref(&packet_data);
                    let timestamp = packet.timestamp / 1000;
                    self.pending_events.push_back(GlassesEvent::AccGyro {
                        accelerometer: Vector3::from_data(nalgebra::ArrayStorage(
                            [packet.accelerometer; 1],
                        )),
                        gyroscope: Vector3::from_data(nalgebra::ArrayStorage(
                            [packet.gyroscope; 1],
                        )),
                        timestamp,
                    });
                    self.pending_events.push_back(GlassesEvent::Magnetometer {
                        magnetometer: Vector3::from_data(nalgebra::ArrayStorage(
                            [packet.magnetometer; 1],
                        )),
                        timestamp,
                    });
                    // NOTE: was always zero on my Max
                    self.handle_key_press(packet.keys_pressed);
                    self.handle_proxy_sensor(packet.proxy_sensor);
                }
                _ => {}
            }
        }
        Ok(self.pending_events.pop_front().unwrap())
    }


    fn get_display_mode(&mut self) -> Result<DisplayMode> {
        let mut result = [0; 0x40];
        self.device_handle.read_control(
            request_type(
                rusb::Direction::In,
                rusb::RequestType::Vendor,
                rusb::Recipient::Device,
            ),
            0x81,
            0x0,
            0x1,
            &mut result,
            TIMEOUT,
        )?;
        match result[1] {
            0 => Ok(DisplayMode::SameOnBoth),
            1 => Ok(DisplayMode::Stereo),
            2 => Ok(DisplayMode::HalfSBS),
            4 => Ok(DisplayMode::HighRefreshRateSBS),
            _ => Ok(DisplayMode::HighRefreshRate),
        }
    }

    fn set_display_mode(&mut self, display_mode: DisplayMode) -> Result<()> {
        let display_mode = match display_mode {
            DisplayMode::SameOnBoth => 0,
            DisplayMode::Stereo => 1,
            DisplayMode::HighRefreshRate => 3,
            DisplayMode::HighRefreshRateSBS => 4,
            _ => return Err(Error::Other("Display mode not supported")),
        };
        self.device_handle.write_control(
            request_type(
                rusb::Direction::Out,
                rusb::RequestType::Vendor,
                rusb::Recipient::Device,
            ),
            0x1,
            display_mode,
            0x1,
            &[0u8; 1],
            TIMEOUT,
        )?;
        Ok(())
    }

    fn display_fov(&self) -> f32 {
        match self.model {
            RokidModel::Air => {
                // 21° is the advertised FOV
                // 20° is the (dynamically) measured one. It works better with normal PD settings
                20f32.to_radians()
            }
            RokidModel::Max => {
                // Measured
                23f32.to_radians()
            }
        }
    }

    fn imu_to_display_matrix(&self, side: Side, ipd: f32) -> Isometry3<f64> {
        let tilt = match self.model {
            RokidModel::Air => 0.022,
            RokidModel::Max => 0.07,
        };
        let ipd = ipd as f64
            * match side {
                Side::Left => -0.5,
                Side::Right => 0.5,
            };
        Translation3::new(ipd, 0.0, 0.0) * UnitQuaternion::from_euler_angles(tilt, 0.0, 0.0)
    }

    fn name(&self) -> &'static str {
        match self.model {
            RokidModel::Air => "Rokid Air",
            RokidModel::Max => "Rokid Max",
        }
    }

    fn display_delay(&self) -> u64 {
        match self.model {
            RokidModel::Air => 15000,
            RokidModel::Max => 13000,
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
struct MiscPacket {
    packet_type: u8,
    seq: u32,
    _unknown_0: [u8; 42],
    keys_pressed: u8,
    _unknown_1: [u8; 3],
    proxy_sensor: u8,
    _unknown_2: [u8; 12],
}

unsafe impl bytemuck::Zeroable for MiscPacket {}
unsafe impl bytemuck::Pod for MiscPacket {}

#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
struct SensorPacket {
    packet_type: u8,
    sensor_type: u8,
    seq: u32,
    _unknown_0: [u8; 3],
    timestamp: u64,
    _unknown_1: [u8; 4],
    vector: [f32; 3],
    _unknown_2: [u8; 31],
}

unsafe impl bytemuck::Zeroable for SensorPacket {}
unsafe impl bytemuck::Pod for SensorPacket {}

#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
struct CombinedPacket {
    packet_type: u8,
    timestamp: u64,
    accelerometer: [f32; 3],
    gyroscope: [f32; 3],
    magnetometer: [f32; 3],
    keys_pressed: u8,
    proxy_sensor: u8,
    _unknown_0: u8,
    vsync_timestamp: u64,
    _unknown_1: [u8; 3],
    display_brightness: u8,
    volume: u8,
    _unknown_2: [u8; 3],
}

unsafe impl bytemuck::Zeroable for CombinedPacket {}
unsafe impl bytemuck::Pod for CombinedPacket {}

impl RokidAir {
    /// Vendor ID of the Rokid Air (Yes, it is 1234. Yes that's probably not very legit)
    pub const VID: u16 = 0x04d2;
    /// Product ID of the Rokid Air
    pub const PID: u16 = 0x162f;

    /// Connect to a specific glasses, based on the two USB fds
    /// Mainly made to work around android permission issues
    #[cfg(target_os = "android")]
    pub fn new(fd: isize) -> Result<Self> {
        use rusb::UsbContext;
        // Do not scan for devices in libusb_init()
        // This is needed on Android, where access to USB devices is limited
        unsafe { rusb::ffi::libusb_set_option(std::ptr::null_mut(), 2) };
        let device_handle = unsafe { GlobalContext::default().open_device_with_fd(fd as i32) }?;
        Self::new_common(device_handle)
    }

    /// Find a connected Rokid Air device and connect to it. (And claim the USB interface)
    /// Only one instance can be alive at a time
    #[cfg(not(target_os = "android"))]
    pub fn new() -> Result<Self> {
        use crate::util::get_device_vid_pid;

        Self::new_common(get_device_vid_pid(Self::VID, Self::PID)?.open()?)
    }

    fn new_common(mut device_handle: DeviceHandle<GlobalContext>) -> Result<Self> {
        device_handle.set_auto_detach_kernel_driver(true)?;

        device_handle.claim_interface(
            get_interface_for_endpoint(&device_handle.device(), INTERRUPT_IN_ENDPOINT).ok_or_else(
                || Error::Other("Could not find endpoint, wrong USB structure (probably)"),
            )?,
        )?;
        let product_string = device_handle
            .read_product_string_ascii(&device_handle.device().device_descriptor()?)?;
        let result = Self {
            device_handle,
            last_accelerometer: None,
            last_gyroscope: None,
            previous_key_states: 0,
            proxy_sensor_was_far: false,
            model: if product_string.contains("Max") {
                RokidModel::Max
            } else {
                RokidModel::Air
            },
            pending_events: Default::default(),
        };
        Ok(result)
    }

    fn handle_key_press(&mut self, keys_pressed: u8) {
        let new_presses = keys_pressed & !self.previous_key_states;
        for bit in 0..8 {
            if new_presses & (1 << bit) != 0 {
                self.pending_events.push_back(GlassesEvent::KeyPress(bit))
            }
        }
        self.previous_key_states = keys_pressed;
    }

    fn handle_proxy_sensor(&mut self, value: u8) {
        let proxy_sensor_is_far = value != 0;
        let send_proxy_event = proxy_sensor_is_far != self.proxy_sensor_was_far;
        self.proxy_sensor_was_far = proxy_sensor_is_far;
        if send_proxy_event {
            self.pending_events.push_back(if proxy_sensor_is_far {
                GlassesEvent::ProximityFar
            } else {
                GlassesEvent::ProximityNear
            });
        }
    }

    /// Read data from the glasses.
    pub fn read_value(&mut self, request:u8, index:u16, value: u16) -> Result<[u8; 0x40]> {
        let expected = 0x40;
        let mut result = [0u8; 0x40]; // Maximum size of any response.
        let rxd = self.device_handle.read_control(
            request_type(
                rusb::Direction::In,
                rusb::RequestType::Vendor,
                rusb::Recipient::Device,
            ),
            request,
            value,
            index,
            &mut result,
            TIMEOUT,
        )?;
        if rxd != expected {
            println!("Expected {} bytes, got {}", expected, rxd);
            return Err(Error::Other("Protocol error"));
        }
        Ok(result)
    }

    /// Write a conbtrol value to the glasses
    pub fn write_value(&mut self, request: u8, index:u16, value: u16, data: &[u8]) -> Result<()> {
        let sent = self.device_handle.write_control(
            request_type(
                rusb::Direction::Out,
                rusb::RequestType::Vendor,
                rusb::Recipient::Device,
            ),
            request,
            value,
            index,
            data,
            TIMEOUT,    
            
        )?;
        if data.len() != sent {
            return Err(Error::WriteFailed);
        }
        Ok(())
    }

    /// * HW Version - 0x81, 0x00, 0x800, [0x10 bytes]
    pub fn hw_version(&mut self) -> String {
        convert_byte_array(self.read_value(0x81, 0x0, 0x800))
    }
    /// * PCBA       - 0x81, 0x00, 0x200, [0x40 bytes]
    pub fn pcba_version(&mut self) -> String {
        convert_byte_array(self.read_value(0x81, 0x0, 0x200))
    }
    /// * Optical ID - 0x81, 0x00, 0x700, [0x40 bytes]
    pub fn optical_id(&mut self) -> String {
        convert_byte_array(self.read_value(0x81, 0x0, 0x700))
    }
    /// * TypeID     - 0x81, 0x00, 0x300, [0x40 bytes]
    pub fn type_id(&mut self) -> String {
        convert_byte_array(self.read_value(0x81, 0x0, 0x300))
    }
    /// * Serial#    - 0x81, 0x00, 0x100, [0x40 bytes]
    pub fn serial_no(&mut self) -> String {
        convert_byte_array(self.read_value(0x81, 0x0, 0x100))
    }
    /// * FW Version - 0x81, 0x00,  0x00, [0x40 bytes]
    pub fn fw_version(&mut self) -> String {
        convert_byte_array(self.read_value(0x81, 0x0, 0x0))
    }
    /// * Seed       - 0x81, 0x00, 0xa00, [0x40 bytes]
    pub fn seed(&mut self) -> String {
        convert_byte_array(self.read_value(0x81, 0x0, 0xa00))
    }

    /// Get the raw display mode
    pub fn get_raw_display_mode(&mut self) -> String {
        convert_data_response(self.read_value(0x81, 0x01, 0x0))
    }

    /// Set the raw display mode
    pub fn set_raw_display_mode(&mut self, mode1:u8, mode2:u8) -> Result<()> {
        self.write_value(0x01, 0x01, mode1.into(), &[mode2])
    }

    /// Get the volume
    pub fn get_volume(&mut self) -> String {
        convert_data_response(self.read_value(0x81, 0x0a, 0x0))
    }

    /// Set the volume
    pub fn set_volume(&mut self, volume: u8) -> Result<()> {
        // Volume must be between 0 and 10.
        let volume = std::cmp::min(std::cmp::max(volume, 0), 10);
        let volume:u16 = (volume * 10).into();
        let data = [0x00];
        self.write_value(0x01, 0x0a, volume.into(), &data)
    }

    /// Get the brightness
    pub fn get_brightness(&mut self) -> String {
        convert_data_response(self.read_value(0x81, 0x02, 0x0))
    }

    /// Set the brightness
    pub fn set_brightness(&mut self, brightness: u8) -> Result<()> {
        // brightness must be between 1 and 6
        let brightness = std::cmp::min(std::cmp::max(brightness, 1), 6);
        let brightness:u16 = match brightness {
            1 => 10,
            2 => 30,
            3 => 45,
            4 => 60,
            5 => 80,
            _ => 100            
        };
        let data = [0x00];
        self.write_value(0x02, 0x02, brightness, &data)
    }
    
}

fn convert_byte_array(byte_array: Result<[u8; 0x40]>) -> String {
    match byte_array {
        Ok(byte_array) => match String::from_utf8(byte_array.to_vec()) {
            Ok(s) => s,
            Err(_) => byte_array.iter().map(|b| format!("{:02X}", b)).collect::<Vec<String>>().join(", ")
        },
        Err(e) => format!("Unknown ({})",e)
    }
}

fn convert_data_response(byte_array: Result<[u8; 0x40]>) -> String {
    match byte_array {
        Ok(byte_array) => byte_array.iter().map(|b| format!("{:02X}", b)).collect::<Vec<String>>().join(", "),
        Err(e) => format!("Unknown ({})",e)
    }
}
