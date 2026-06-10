// The `#[characteristic]` macro expands the NUS fields into a borrow clippy flags as needless;
// the warning lands in generated code, so it can only be silenced at module scope.
#![allow(clippy::needless_borrows_for_generic_args)]

use embassy_futures::join::join;
use heapless::Vec;
use static_cell::StaticCell;
use trouble_host::prelude::*;

const CONNECTIONS_MAX: usize = 1;
// TODO: verify this value matches trouble-host's channel counting convention
const L2CAP_CHANNELS_MAX: usize = 2;
const HCI_SLOTS: usize = 10;

const DEVICE_NAME: &str = "LineFollower";

#[gatt_server]
struct NusServer {
    nus: NusService,
}

/// NUS (Nordic UART Service): TX carries log output; RX is declared but unused.
#[gatt_service(uuid = "6e400001-b5a3-f393-e0a9-e50e24dcca9e")]
struct NusService {
    #[characteristic(uuid = "6e400003-b5a3-f393-e0a9-e50e24dcca9e", notify)]
    tx: Vec<u8, 244>,
    #[characteristic(uuid = "6e400002-b5a3-f393-e0a9-e50e24dcca9e", write_without_response)]
    rx: Vec<u8, 244>,
}

type BleResources = HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX>;
type BleStack = Stack<'static, lf_hal::BleController<HCI_SLOTS>, DefaultPacketPool>;

static HOST_RESOURCES: StaticCell<BleResources> = StaticCell::new();
static BLE_STACK: StaticCell<BleStack> = StaticCell::new();
static NUS_SERVER: StaticCell<NusServer<'static>> = StaticCell::new();

/// Spawns the BLE task, taking ownership of the radio via `hal`.
///
/// # Panics
/// Panics if called more than once (the BLE task can only be spawned once, and
/// [`lf_hal::Hal::init_bt_controller`] likewise rejects a second call).
pub fn init_ble(spawner: embassy_executor::Spawner, hal: &mut lf_hal::Hal<'static>) {
    spawner.spawn(ble_task(hal.init_bt_controller::<HCI_SLOTS>()).unwrap());
}

#[embassy_executor::task]
async fn ble_task(controller: lf_hal::BleController<HCI_SLOTS>) -> ! {
    let resources = HOST_RESOURCES.init(BleResources::new());
    let stack = BLE_STACK.init(trouble_host::new(controller, resources));
    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();

    let server = NUS_SERVER.init(
        NusServer::new_with_config(GapConfig::Peripheral(PeripheralConfig {
            name: DEVICE_NAME,
            appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
        }))
        .unwrap(),
    );

    let _ = join(runner_loop(runner), async move {
        loop {
            match advertise(&mut peripheral, &*server).await {
                Ok(conn) => {
                    log::info!("BLE connected");
                    nus_task(&*server, &conn).await;
                    log::info!("BLE disconnected");
                }
                Err(e) => log::warn!("BLE advertise error: {e:?}"),
            }
        }
    })
    .await;

    unreachable!()
}

async fn runner_loop<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            log::error!("BLE runner error: {e:?}");
        }
    }
}

async fn advertise<'a, C: Controller>(
    peripheral: &mut Peripheral<'a, C, DefaultPacketPool>,
    server: &'a NusServer<'a>,
) -> Result<GattConnection<'a, 'a, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut adv_data = [0u8; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(DEVICE_NAME.as_bytes()),
        ],
        &mut adv_data[..],
    )?;
    log::info!("BLE advertising");
    let advertiser = peripheral
        .advertise(
            &AdvertisementParameters::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &adv_data[..len],
                scan_data: &[],
            },
        )
        .await?;
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    Ok(conn)
}

async fn nus_task<P: PacketPool>(server: &NusServer<'_>, conn: &GattConnection<'_, '_, P>) {
    use embassy_futures::select::{Either, select};
    loop {
        match select(crate::ble_logger::read_log_bytes(), conn.next()).await {
            Either::First(chunk) => {
                if server.nus.tx.notify(conn, &chunk).await.is_err() {
                    break;
                }
            }
            Either::Second(GattConnectionEvent::Disconnected { .. }) => break,
            Either::Second(GattConnectionEvent::Gatt { event }) => {
                let _ = event.accept();
            }
            Either::Second(_) => {}
        }
    }
}
