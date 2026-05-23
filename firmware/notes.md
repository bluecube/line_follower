  Key non-obvious things:
  - [u8; 244] doesn't implement AsGatt; heapless::Vec<u8, 244> does with variable-length serialization — and
  DefaultPacketPool already defaults to MTU 251, so no custom pool needed
  - Stack::build(&'stack self) borrows stack rather than returning owned values, so resources AND the stack
  itself need to be in StaticCell for the peripheral/server to have 'static lifetimes
  - The NUS server must also go in StaticCell (NUS_SERVER) so async move can hold &'static NusServer<'static>
  — this is what finally satisfies the drop checker for the proc-macro-generated NusServer<'_> type
  - embassy-sync needed a downgrade from 0.8 to 0.7 because the #[gatt_server] macro generates code using
  trouble-host's embassy-sync 0.7 types, which are incompatible with 0.8
