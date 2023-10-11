// Copyright (C) 2023, Steven Johnson, Alex Badics
// This file is part of ar-drivers-rs
// Licensed under the MIT license. See LICENSE file in the project root for details.

use ar_drivers::rokid;
use clap::Parser;

/// Setup AR glasses modes
#[derive(clap::Parser, Debug)]
struct CliArgs {
    /// Display mode
    #[clap(long, value_enum)]
    mode: Option<CliDisplayMode>,

    /// Volume to set (0-10), 
    #[clap(long)]
    vol: Option<u8>,

    /// Brightness to set (1-6), 
    #[clap(long)]
    brightness: Option<u8>,
    
    /// Don't display anything.
    #[clap(long, short)]
    silent: bool,
}

#[derive(clap::ValueEnum, Debug, Clone, Copy)]
enum CliDisplayMode {
    /// 1920x1080 with Referesh Rates 60/50/48/40 Hz
    #[value(name("1920x1080-60hz"), alias("1920x1080-50hz"), alias("1920x1080-48hz"), alias("1920x1080-40hz"))]
    X1920x1080_60,
    /// 1920*1080 60Hz SBS, (3840x1080 L|R)
    #[value(name("1920x1080-60hz-SBS"))]
    X1920x1080_60Sbs,
    /// 960*1080 60Hz SBS, (1920x1080 L|R)
    #[value(name("960x1080-60hz-SBS"))]
    X960x1080_60Sbs,
    /// 1920x1200 with Referesh Rates 60/120 Hz
    #[value(name("1920x1200-60hz"), alias("1920x1080-120hz"))]
    X1920x1200_60,
    /// 1920*1200 90Hz SBS, (3840x1200 L|R)
    #[value(name("1920x1200-90hz-SBS"))]
    X1920x1200_90Sbs,
    /// 1920x1200 SBS with Referesh Rates 60/50/48/40 Hz (3840x1200 L|R)
    #[value(name("1920x1200-60hz-SBS"), alias("1920x1200-50hz-SBS"), alias("1920x1200-48hz-SBS"), alias("1920x1200-40hz-SBS"))]
    X1920x1200_60Sbs,
}

impl CliDisplayMode {
    pub fn mode(&self) -> u8 {
        match self {
            CliDisplayMode::X1920x1080_60 => 0,
            CliDisplayMode::X1920x1080_60Sbs => 1,
            CliDisplayMode::X960x1080_60Sbs => 2,
            CliDisplayMode::X1920x1200_60 => 3,
            CliDisplayMode::X1920x1200_60Sbs => 4,
            CliDisplayMode::X1920x1200_90Sbs => 5,
        }
    }
}

fn main() {
    let args = CliArgs::parse();
    
    let mut glasses = match rokid::RokidAir::new() {
        Ok(glasses) => glasses,
        Err(e) => {
            println!("Failed to connect to glasses: {}", e);
            return;
        }
    };

    if !args.silent {
        println!("HW Version  : {}", glasses.hw_version());
        println!("PCBA Version: {}", glasses.pcba_version());
        println!("Optical ID  : {}", glasses.optical_id());
        println!("TypeID      : {}", glasses.type_id());
        println!("Serial#     : {}", glasses.serial_no());
        println!("FW Version  : {}", glasses.fw_version());
        println!("Seed        : {}", glasses.seed());

        println!();
        println!("Display mode: {}", glasses.get_raw_display_mode());
        println!("Volume      : {}", glasses.get_volume());
        println!("Brightness  : {}", glasses.get_brightness());
    }

    if args.vol.is_some() {
        if let Err(e) = glasses.set_volume(args.vol.unwrap()) {
            println!("Failed to set volume: {}", e);
        }
    }

    if args.brightness.is_some() {
        if let Err(e) = glasses.set_brightness(args.brightness.unwrap()) {
            println!("Failed to set brightness: {}", e);
        }
    }

    if args.mode.is_some() {
        if let Err(e) = glasses.set_raw_display_mode(args.mode.unwrap().mode(), 0) {
            println!("Failed to set mode {}: {}", args.mode.unwrap().mode(), e);
        }
    }
}


