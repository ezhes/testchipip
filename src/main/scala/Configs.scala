package testchipip

import chisel3._
import freechips.rocketchip.system.BaseConfig
import org.chipsalliance.cde.config.{Parameters, Config}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.diplomacy.{AsynchronousCrossing, ClockCrossingType, AddressSet}
import freechips.rocketchip.unittest.UnitTests
import sifive.blocks.devices.uart.{UARTParams}

class WithRingSystemBus(
    buffer: TLNetworkBufferParams = TLNetworkBufferParams.default)
    extends Config((site, here, up) => {
  case TLNetworkTopologyLocated(InSubsystem) =>
    up(TLNetworkTopologyLocated(InSubsystem), site).map(topo =>
      topo match {
        case j: JustOneBusTopologyParams =>
          new TLBusWrapperTopology(j.instantiations.map(inst => inst match {
            case (SBUS, sbus_params: SystemBusParams) => (SBUS, RingSystemBusParams(sbus_params, buffer))
            case a => a
          }
        ), j.connections)
        case x => x
      }
    )
})

class WithTestChipUnitTests extends Config((site, here, up) => {
  case UnitTests => (testParams: Parameters) =>
    TestChipUnitTests(testParams)
})

class WithClockUtilTests extends Config((site, here, up) => {
  case UnitTests => (testParams: Parameters) => ClockUtilTests()
})

class TestChipUnitTestConfig extends Config(
  new WithTestChipUnitTests ++ new BaseConfig)

class ClockUtilTestConfig extends Config(
  new WithClockUtilTests ++ new BaseConfig)

class WithBlockDevice(enable: Boolean = true) extends Config((site, here, up) => {
  case BlockDeviceKey => enable match {
    case true => Some(BlockDeviceConfig())
    case false => None
  }
})

class WithBlockDeviceLocations(slaveWhere: TLBusWrapperLocation = PBUS, masterWhere: TLBusWrapperLocation = FBUS) extends Config((site, here, up) => {
  case BlockDeviceAttachKey => BlockDeviceAttachParams(slaveWhere, masterWhere)
})

class WithNBlockDeviceTrackers(n: Int) extends Config((site, here, up) => {
  case BlockDeviceKey => up(BlockDeviceKey, site) match {
    case Some(a) => Some(a.copy(nTrackers = n))
    case None => None
  }
})

// Default size should be tiny
class WithDefaultSerialTL extends Config((site, here, up) => {
  case SerialTLKey => Some(SerialTLParams())
})

class WithSerialTLWidth(width: Int) extends Config((site, here, up) => {
  case SerialTLKey => up(SerialTLKey).map(k => k.copy(width=width))
})

class WithAXIMemOverSerialTL(axiMemOverSerialTLParams: AXIMemOverSerialTLClockParams) extends Config((site, here, up) => {
  case SerialTLKey => up(SerialTLKey).map(s => s.copy(serialTLManagerParams=s.serialTLManagerParams.map(
    _.copy(axiMemOverSerialTLParams=Some(axiMemOverSerialTLParams)))))
})

class WithSerialTLMasterLocation(masterWhere: TLBusWrapperLocation) extends Config((site, here, up) => {
  case SerialTLKey => up(SerialTLKey).map(s => s.copy(attachParams=s.attachParams.copy(masterWhere = masterWhere)))
})

class WithSerialTLSlaveLocation(slaveWhere: TLBusWrapperLocation) extends Config((site, here, up) => {
  case SerialTLKey => up(SerialTLKey).map(s => s.copy(attachParams=s.attachParams.copy(slaveWhere = slaveWhere)))
})

class WithSerialTLPBusManager extends WithSerialTLSlaveLocation(PBUS)

class WithSerialSlaveCrossingType(xType: ClockCrossingType) extends Config((site, here, up) => {
  case SerialTLKey => up(SerialTLKey).map(s => s.copy(attachParams=s.attachParams.copy(slaveCrossingType = xType)))
})

class WithAsynchronousSerialSlaveCrossing extends WithSerialSlaveCrossingType(AsynchronousCrossing())

class WithSerialTLMem(
  base: BigInt = BigInt("80000000", 16),
  size: BigInt = BigInt("10000000", 16),
  idBits: Int = 8,
  isMainMemory: Boolean = true
) extends Config((site, here, up) => {
  case SerialTLKey => {
    val masterPortParams = MasterPortParams(
      base = base,
      size = size,
      idBits = idBits,
      beatBytes = site(MemoryBusKey).beatBytes
    )
    up(SerialTLKey, site).map { k => k.copy(
      serialTLManagerParams = Some(k.serialTLManagerParams.getOrElse(SerialTLManagerParams(memParams = masterPortParams))
        .copy(memParams = masterPortParams, isMemoryDevice = isMainMemory)
      )
    )}
  }
})


class WithSerialTLBackingMemory extends Config((site, here, up) => {
  case ExtMem => None
  case SerialTLKey => up(SerialTLKey, site).map { k => k.copy(
    serialTLManagerParams = Some(k.serialTLManagerParams.getOrElse(SerialTLManagerParams(memParams = up(ExtMem).get.master))
      .copy(memParams = up(ExtMem).get.master, isMemoryDevice = true))
  )}
})

class WithSerialTLROM extends Config((site, here, up) => {
  case SerialTLKey => up(SerialTLKey, site).map { k => k.copy(
    serialTLManagerParams = k.serialTLManagerParams.map { s => s.copy(
      romParams = Some(SerialTLROMParams())
    )}
  )}
})

class WithSerialTLROMFile(file: String) extends Config((site, here, up) => {
  case SerialTLKey => up(SerialTLKey, site).map { k => k.copy(
    serialTLManagerParams = k.serialTLManagerParams.map { s => s.copy(
      romParams = s.romParams.map(_.copy(contentFileName = Some(file)))
    )}
  )}
})

class WithSerialTLClientIdBits(bits: Int) extends Config((site, here, up) => {
  case SerialTLKey => up(SerialTLKey).map { k => k.copy(clientIdBits = bits) }
})

class WithSerialTLClockDirection(provideClockFreqMHz: Option[Int] = None) extends Config((site, here, up) => {
  case SerialTLKey => up(SerialTLKey).map(_.copy(provideClockFreqMHz = provideClockFreqMHz))
})

class WithNoSerialTL extends Config((site, here, up) => {
  case SerialTLKey => None
})

class WithUARTTSITLClient(initBaudRate: BigInt = BigInt(115200)) extends Config((site, here, up) => {
  case UARTTSITLClientKey => Some(UARTTSITLClientParams(UARTParams(0, initBaudRate=initBaudRate)))
})

class WithOffchipBus extends Config((site, here, up) => {
  case TLNetworkTopologyLocated(InSubsystem) => up(TLNetworkTopologyLocated(InSubsystem)) :+
    OffchipBusTopologyParams(SystemBusParams(beatBytes = 8, blockBytes = site(CacheBlockBytes)))
})

class WithOffchipBusManager(
  location: TLBusWrapperLocation,
  blockRange: Seq[AddressSet] = Nil,
  replicationBase: Option[BigInt] = None) extends Config((site, here, up) => {
    case TLNetworkTopologyLocated(InSubsystem) => up(TLNetworkTopologyLocated(InSubsystem)) :+
      OffchipBusTopologyConnectionParams(location, blockRange, replicationBase)
})

class WithTilesStartInReset(harts: Int*) extends Config((site, here, up) => {
  case TileResetCtrlKey => up(TileResetCtrlKey, site).copy(initResetHarts = up(TileResetCtrlKey, site).initResetHarts ++ harts)
})

class WithBootAddrReg(params: BootAddrRegParams = BootAddrRegParams()) extends Config((site, here, up) => {
  case BootAddrRegKey => Some(params)
})

class WithNoBootAddrReg extends Config((site, here, up) => {
  case BootAddrRegKey => None
})

class WithCustomBootPin(params: CustomBootPinParams = CustomBootPinParams()) extends Config((site, here, up) => {
  case CustomBootPinKey => Some(params)
})

class WithCustomBootPinAltAddr(address: BigInt) extends Config((site, here, up) => {
  case CustomBootPinKey => up(CustomBootPinKey, site).map(p => p.copy(customBootAddress = address))
})

class WithNoCustomBootPin extends Config((site, here, up) => {
  case CustomBootPinKey => None
})
