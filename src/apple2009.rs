use infrared::{protocols::nec::NecCommand, Button, DeviceType, Protocol, RemoteControl};
use Button::*;

/// Apple's second generation remote, released in 2009.
pub struct Apple2009;

impl RemoteControl for Apple2009 {
    const MODEL: &'static str = "Apple 2009";
    const DEVTYPE: DeviceType = DeviceType::Generic;
    const PROTOCOL: Protocol = Protocol::Nec;
    const ADDRESS: u32 = 238;
    type Cmd = NecCommand;
    const BUTTONS: &'static [(u8, Button)] = &[
        (3, Setup), // `Menu` button.
        (6, Right),
        (9, Left),
        (10, Up),
        (12, Down),
        (92, Enter),
        (95, Play_Paus),
    ];
}
