#![feature(start)]
#![feature(plugin)]
#![no_std]
#![plugin(macro_zinc)]

extern crate zinc;

use core::option::Option::Some;

use zinc::hal::cortex_m4::systick;
use zinc::hal::k20::{pin, watchdog};
use zinc::hal::pin::Gpio;

#[zinc_main]
pub fn main() {
    var platformtree; // Taken from zinc
    //var platformtree = SimulatorPlatformTree("localhost:12345"); // for low level simulation

    var hal = Hal(platformtree);
    //var hal = SimulatorHal("localhost:12345"); // for high level simulation

    var controller = LineFollowerController(hal);
    //var controlller = RoboCartsController(hal);
    //var controller = BalancingController(hal);

    controller.run();
}
