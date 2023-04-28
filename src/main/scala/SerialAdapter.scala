package testchipip

import chisel3._
import chisel3.util._
import chisel3.experimental.{IO, DataMirror}
import org.chipsalliance.cde.config.{Parameters, Field}
import freechips.rocketchip.subsystem._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.devices.debug.HasPeripheryDebug
import freechips.rocketchip.devices.tilelink._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._
import freechips.rocketchip.prci._
import scala.math.min
import freechips.rocketchip.amba.axi4._
import sifive.blocks.devices.uart._

import java.nio.ByteBuffer
import java.nio.file.{Files, Paths}

case object SerialAdapter {
  def connectHarnessRAM(serdesser: TLSerdesser, port: SerialIO, reset: Reset): SerialRAM = {
    implicit val p: Parameters = serdesser.p

    val ram = LazyModule(new SerialRAM(serdesser))

    val module = Module(ram.module)
    module.io.ser <> port

    require(ram.serdesser.module.mergedParams == serdesser.module.mergedParams,
      "Mismatch between chip-side diplomatic params and harness-side diplomatic params:\n" +
      s"Harness-side params: ${ram.serdesser.module.mergedParams}\n" +
      s"Chip-side params: ${serdesser.module.mergedParams}")

    ram
  }

  def connectHarnessMultiClockAXIRAM(serdesser: TLSerdesser, serial_port: SerialIO, mem_clock_port: ClockBundle, reset: Reset): MultiClockSerialAXIRAM = {
    implicit val p: Parameters = serdesser.p

    val ram = LazyModule(new MultiClockSerialAXIRAM(serdesser))

    val module = Module(ram.module)
    module.io.ser <> serial_port
    module.io.passthrough_clock_reset <> mem_clock_port

    require(ram.serdesser.module.mergedParams == serdesser.module.mergedParams,
      "Mismatch between chip-side diplomatic params and harness-side diplomatic params:\n" +
      s"Harness-side params: ${ram.serdesser.module.mergedParams}\n" +
      s"Chip-side params: ${serdesser.module.mergedParams}")

    ram
  }

  def tieoff(serial: Option[SerialIO]) {
    serial.foreach { s =>
      s.in.valid := false.B
      s.in.bits := DontCare
      s.out.ready := true.B
    }
  }

  def tieoff(serial: SerialIO) { tieoff(Some(serial)) }
}

object SerialTLROM {
  def apply(romParams: SerialTLROMParams, beatBytes: Int)(implicit p: Parameters): TLROM = {
    lazy val romContents = {
      val romData = romParams.contentFileName.map(n => Files.readAllBytes(Paths.get(n))).getOrElse(
        Array(
          0x1b, 0x05, 0x10, 0x00, // 0010051b     addiw    a0,zero,1
          0x13, 0x15, 0xf5, 0x01, // 01f51513     slli     a0,a0,0x1f (li a0, 0x8000_0000)
          0x73, 0x10, 0x15, 0x34, // 34151073     csrw     mepc,a0
          0x37, 0x25, 0x00, 0x00, // 00002537     lui      a0,0x2
          0x1b, 0x05, 0x05, 0x80, // 8005051b     addiw    a0,a0,-2048
          0x73, 0x20, 0x05, 0x30, // 30052073     csrs     mstatus,a0
          0x73, 0x25, 0x40, 0xf1, // f1402573     csrr     a0,mhartid
          0x73, 0x00, 0x20, 0x30  // 30200073     mret
        ).map(_.toByte)
      )
      val rom = ByteBuffer.wrap(romData)
      rom.array()
    }
    val rom = LazyModule(new TLROM(romParams.address, romParams.size, romContents, true, beatBytes))
    rom
  }
}

class SerialRAM(tl_serdesser: TLSerdesser)(implicit p: Parameters) extends LazyModule {
  val managerParams = tl_serdesser.module.client_edge.map(_.slave) // the managerParams are the chip-side clientParams
  val clientParams = tl_serdesser.module.manager_edge.map(_.master) // The clientParams are the chip-side managerParams
  val tsi2tl = LazyModule(new TSIToTileLink)
  val serdesser = LazyModule(new TLSerdesser(
    tl_serdesser.w,
    clientParams,
    managerParams
  ))

  serdesser.clientNode.foreach { clientNode =>
    val memParams = p(SerialTLKey).get.serialManagerParams.get.memParams
    val romParams = p(SerialTLKey).get.serialManagerParams.get.romParams
    val srams = AddressSet.misaligned(memParams.base, memParams.size).map { aset =>
      LazyModule(new TLRAM(
        aset,
        beatBytes = memParams.beatBytes
      ))
    }

    val xbar = TLXbar()
    srams.foreach { s => s.node := TLBuffer() := TLFragmenter(memParams.beatBytes, p(CacheBlockBytes)) := xbar }

    romParams.map { romParams =>
      val rom = SerialTLROM(romParams, memParams.beatBytes)
      rom.node := TLFragmenter(memParams.beatBytes, p(CacheBlockBytes)) := xbar
    }

    xbar := clientNode
  }

  serdesser.managerNode.get := TLBuffer() := tsi2tl.node

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val io = IO(new Bundle {
      val ser = Flipped(new SerialIO(tl_serdesser.w))
      val tsi = new TSIIO
      val tsi2tl_state = Output(UInt())
    })

    serdesser.module.io.ser.in <> io.ser.out
    io.ser.in <> serdesser.module.io.ser.out
    io.tsi <> tsi2tl.module.io.tsi
    io.tsi2tl_state := tsi2tl.module.io.state
  }
}

class MultiClockSerialAXIRAM(tl_serdesser: TLSerdesser)(implicit p: Parameters) extends LazyModule {

  // setup serdes and serial tsi2tl
  val managerParams = tl_serdesser.module.client_edge.map(_.slave) // the managerParams are the chip-side clientParams
  val clientParams = tl_serdesser.module.manager_edge.map(_.master) // The clientParams are the chip-side managerParams
  val tsi2tl = LazyModule(new TSIToTileLink)
  val serdesser = LazyModule(new TLSerdesser(
    tl_serdesser.w,
    clientParams,
    managerParams
  ))

  // connect the tsi2tl to tlserdesser
  serdesser.managerNode.get := TLBuffer() := tsi2tl.node

  val memClkRstDomain = LazyModule(new ClockSinkDomain(name=Some("mem-over-serialtl-domain")))
  val memClkRstSource = ClockSourceNode(Seq(ClockSourceParameters()))
  memClkRstDomain.clockNode := memClkRstSource

  val (mem_axi4, memNode) = serdesser.clientNode.map { clientNode =>
    val axiMemOverSerialTLParams = p(SerialTLKey).get.serialManagerParams.get.axiMemOverSerialTLParams.get
    val memCrossing = axiMemOverSerialTLParams.axiClockParams.map(_.crossingType).getOrElse(SynchronousCrossing())
    val memParams = p(SerialTLKey).get.serialManagerParams.get.memParams
    val romParams = p(SerialTLKey).get.serialManagerParams.get.romParams

    val memXbar = memClkRstDomain { TLXbar() }
    romParams.map { romParams =>
      val rom = memClkRstDomain { SerialTLROM(romParams, memParams.beatBytes) }
      memClkRstDomain {
        rom.node := TLFragmenter(memParams.beatBytes, p(CacheBlockBytes)) := memXbar
      }
    }
    (memClkRstDomain.crossIn(memXbar)(ValName("MemPortCrossing")))(memCrossing) := clientNode

    // TODO: Currently only supports single-channel memory
    val memPortParamsOpt = Some(MemoryPortParams(memParams, 1))
    val portName = "axi4"
    val device = new MemoryDevice
    val idBits = memPortParamsOpt.map(_.master.idBits).getOrElse(1)
    val memBusParams = p(MemoryBusKey)

    val memNode = memClkRstDomain {
      AXI4SlaveNode(memPortParamsOpt.map({ case MemoryPortParams(memPortParams, nMemoryChannels, _) =>
        Seq.tabulate(nMemoryChannels) { channel =>
          val base = AddressSet.misaligned(memPortParams.base, memPortParams.size)
          val filter = AddressSet(channel * memBusParams.blockBytes, ~((nMemoryChannels-1) * memBusParams.blockBytes))

          AXI4SlavePortParameters(
            slaves = Seq(AXI4SlaveParameters(
              address       = base.flatMap(_.intersect(filter)),
              resources     = device.reg,
              regionType    = RegionType.UNCACHED, // cacheable
              executable    = true,
              supportsWrite = TransferSizes(1, memBusParams.blockBytes),
              supportsRead  = TransferSizes(1, memBusParams.blockBytes),
            interleavedId = Some(0))), // slave does not interleave read responses
            beatBytes = memPortParams.beatBytes)
        }
      }).toList.flatten)
    }

    val memPort = memClkRstDomain {
      // connect axi mem to axi widgets to mem xbar
      (memNode
        :*= AXI4UserYanker()
        :*= AXI4IdIndexer(idBits)
        :*= TLToAXI4()
        :*= TLWidthWidget(memBusParams.beatBytes)
        :*= memXbar)

      InModuleBody { memNode.makeIOs() }
    }

    val mem_axi4 = InModuleBody {
      val ports: Seq[ClockedAndResetIO[AXI4Bundle]] = memPort.zipWithIndex.map({ case (m, i) =>
        val port = IO(new ClockedAndResetIO(DataMirror.internal.chiselTypeClone[AXI4Bundle](m))).suggestName(s"axi4_mem_${i}")
        port.bits <> m
        port.clock := memClkRstSource.out.head._1.clock
        port.reset := memClkRstSource.out.head._1.reset
        port
      }).toSeq
      ports
    }
    (mem_axi4, memNode)
  }.unzip

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) {
    val io = IO(new Bundle {
      val ser = Flipped(new SerialIO(tl_serdesser.w))
      val tsi = new TSIIO
      val passthrough_clock_reset = Flipped(new ClockBundle(ClockBundleParameters()))
    })

    // setup clock domain
    val axiMemOverSerialTLParams = p(SerialTLKey).get.serialManagerParams
      .map(_.axiMemOverSerialTLParams).flatten.getOrElse(AXIMemOverSerialTLClockParams())
    axiMemOverSerialTLParams.axiClockParams match {
      case Some(params) => {
        // setup the clock domain to be the passthrough clock
        memClkRstSource.out.head._1 <> io.passthrough_clock_reset
      }
      case None => {
        // connect to implicit clock/reset
        memClkRstSource.out.head._1.clock <> clock
        memClkRstSource.out.head._1.reset <> reset
      }
    }

    // connect the tl-serdes and tsi2tl r in the serdes clock domain
    serdesser.module.io.ser.in <> io.ser.out
    io.ser.in <> serdesser.module.io.ser.out
    io.tsi <> tsi2tl.module.io.tsi
  }
}

class SerialWidthAdapter(narrowW: Int, wideW: Int) extends Module {
  require(wideW > narrowW)
  require(wideW % narrowW == 0)
  val io = IO(new Bundle {
    val narrow = new SerialIO(narrowW)
    val wide = new SerialIO(wideW)
  })

  val beats = wideW / narrowW

  val narrow_beats = RegInit(0.U(log2Ceil(beats).W))
  val narrow_last_beat = narrow_beats === (beats-1).U
  val narrow_data = Reg(Vec(beats-1, UInt(narrowW.W)))

  val wide_beats = RegInit(0.U(log2Ceil(beats).W))
  val wide_last_beat = wide_beats === (beats-1).U

  io.narrow.in.ready := Mux(narrow_last_beat, io.wide.out.ready, true.B)
  when (io.narrow.in.fire()) {
    narrow_beats := Mux(narrow_last_beat, 0.U, narrow_beats + 1.U)
    when (!narrow_last_beat) { narrow_data(narrow_beats) := io.narrow.in.bits }
  }
  io.wide.out.valid := narrow_last_beat && io.narrow.in.valid
  io.wide.out.bits := Cat(io.narrow.in.bits, narrow_data.asUInt)

  io.narrow.out.valid := io.wide.in.valid
  io.narrow.out.bits := io.wide.in.bits >> (wide_beats << 3)
  when (io.narrow.out.fire()) {
    wide_beats := Mux(wide_last_beat, 0.U, wide_beats + 1.U)
  }
  io.wide.in.ready := wide_last_beat && io.narrow.out.ready
}
