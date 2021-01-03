#![no_main]
#![no_std]

use debouncr::Debouncer;
use embedded_hal::digital::v2::*;
use rtic::app;
use rtic::cyccnt::{Duration, U32Ext};
use rtt_target::rprintln;
use stm32f1xx_hal::{gpio::*, pac, prelude::*, rcc::Clocks, timer};

const SYSCLK: u32 = 72_000_000;
const BUTTON_POLL_PERIOD: u32 = SYSCLK / 100; // Hz
const CYCLES_PER_STEP: u32 = 1_000_000;
const MAX_STEPS: u32 = 10;

#[app(device = stm32f1xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        #[init(1)]
        steps: u32,
        tim2: timer::CountDownTimer<pac::TIM2>,
        led: gpioc::PC13<Output<PushPull>>,
        button: gpioa::PA10<Input<PullUp>>,
        button_state: Debouncer<u8, debouncr::Repeat6>,
        scope: gpioa::PA4<Output<PushPull>>,
    }

    #[init(spawn = [blink_led, poll_button])]
    fn init(cx: init::Context) -> init::LateResources {
        rtt_target::rtt_init_print!();
        rprintln!("RTIC init started");
        let mut cp = cx.core;
        let dp = cx.device;

        // Enable cycle counter; used for scheduling.
        cp.DWT.enable_cycle_counter();

        // Setup and apply clock confiugration.
        let mut flash = dp.FLASH.constrain();
        let mut rcc = dp.RCC.constrain();
        let clocks: Clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(SYSCLK.hz())
            .pclk1((SYSCLK / 2).hz())
            .freeze(&mut flash.acr);
        rprintln!(" SYSCLK: {:?} MHz", clocks.sysclk().0 / 1_000_000);
        rprintln!(" HCLK: {:?} MHz", clocks.hclk().0 / 1_000_000);
        rprintln!(" APB1 clk: {:?} MHz", clocks.pclk1().0 / 1_000_000);
        rprintln!(" APB1 TIM: {:?} MHz", clocks.pclk1_tim().0 / 1_000_000);
        rprintln!(" APB2 clk: {:?} MHz", clocks.pclk2().0 / 1_000_000);
        rprintln!(" ADCCLK: {:?} MHz", clocks.adcclk().0 / 1_000_000);

        // Timer setup.
        let mut tim2: timer::CountDownTimer<pac::TIM2> =
            timer::Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1).start_count_down(2.khz());
        tim2.listen(timer::Event::Update);

        // Peripheral setup.
        let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
        let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

        // Configure pc13 as output via CR high register.
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        led.set_high().unwrap(); // LED off

        // Configure pa4 as output for oscilloscope.
        let mut scope = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);
        scope.set_low().unwrap(); // Oscill low

        // Setup pa10 button interrupt.
        let button = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);

        cx.spawn.poll_button().unwrap();
        cx.spawn.blink_led().unwrap();

        // Prevent wait-for-interrupt (default rtic idle) from stalling debug features.
        //
        // See: https://github.com/probe-rs/probe-rs/issues/350
        dp.DBGMCU.cr.modify(|_, w| {
            w.dbg_sleep().set_bit();
            w.dbg_standby().set_bit();
            w.dbg_stop().set_bit()
        });
        let _dma1 = dp.DMA1.split(&mut rcc.ahb);

        rprintln!("RTIC init completed");

        init::LateResources {
            led,
            tim2,
            button,
            button_state: debouncr::debounce_6(false),
            scope,
        }
    }

    #[task(resources = [steps, led], schedule = [blink_led])]
    fn blink_led(cx: blink_led::Context) {
        let blink_led::Resources { mut steps, led } = cx.resources;

        led.toggle().unwrap();

        // Schedule next blink.
        let steps = steps.lock(|s| *s);
        let delay = Duration::from_cycles(CYCLES_PER_STEP * steps);
        cx.schedule.blink_led(cx.scheduled + delay).unwrap();
    }

    #[task(binds = TIM2, priority = 3, resources = [tim2, scope])]
    fn toggle_scope(cx: toggle_scope::Context) {
        let toggle_scope::Resources { tim2, scope } = cx.resources;

        scope.toggle().unwrap();
        tim2.clear_update_interrupt_flag();
    }

    #[task(
        priority = 2,
        resources = [button, button_state],
        spawn = [button_press],
        schedule = [poll_button]
    )]
    fn poll_button(cx: poll_button::Context) {
        // Button is active low.
        let pressed = cx.resources.button.is_low().unwrap();
        let edge = cx.resources.button_state.update(pressed);
        if edge == Some(debouncr::Edge::Rising) {
            cx.spawn.button_press().unwrap();
        }

        // Schedule next button poll.
        cx.schedule
            .poll_button(cx.scheduled + BUTTON_POLL_PERIOD.cycles())
            .unwrap();
    }

    #[task(priority = 2, resources = [steps])]
    fn button_press(cx: button_press::Context) {
        let steps = *cx.resources.steps;
        let new_steps = (steps % MAX_STEPS) + 1;
        *cx.resources.steps = new_steps;

        rprintln!("steps: {} -> {}", steps, new_steps);
    }

    // Unused interrupts for task scheduling.
    extern "C" {
        fn SPI1();
        fn SPI2();
    }
};

#[inline(never)]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {}
}
