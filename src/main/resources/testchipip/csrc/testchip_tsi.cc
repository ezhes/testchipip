#include "testchip_tsi.h"
#include <stdexcept>

testchip_tsi_t::testchip_tsi_t(int argc, char** argv, bool can_have_loadmem) : tsi_t(argc, argv)
{
  has_loadmem = false;
  init_accesses = std::vector<init_access_t>();
  write_hart0_msip = true;
  is_loadmem = false;
  coherent_offset = 0;
  coherent_base = 0;
  coherent_size = 0;
  std::vector<std::string> args(argv + 1, argv + argc);
  for (auto& arg : args) {
    if (arg.find("+loadmem=") == 0)
      has_loadmem = can_have_loadmem;
    if (arg.find("+init_write=0x") == 0) {
      auto d = arg.find(":0x");
      if (d == std::string::npos) {
        throw std::invalid_argument("Improperly formatted +init_write argument");
      }
      uint64_t addr = strtoull(arg.substr(14, d - 14).c_str(), 0, 16);
      uint32_t val = strtoull(arg.substr(d + 3).c_str(), 0, 16);
      init_access_t access = { .address=addr, .stdata=val, .store=true };
      init_accesses.push_back(access);
    }
    if (arg.find("+init_read=0x") == 0) {
      uint64_t addr = strtoull(arg.substr(13).c_str(), 0, 16);
      init_access_t access = { .address=addr, .stdata=0, .store=false };
      init_accesses.push_back(access);
    }
    if (arg.find("+no_hart0_msip") == 0)
      write_hart0_msip = false;
    if (arg.find("+coh_offset=0x") == 0)
      coherent_offset = strtoull(arg.substr(14).c_str(), 0, 16);
    if (arg.find("+coh_base=0x") == 0)
      coherent_base = strtoull(arg.substr(12).c_str(), 0, 16);
    if (arg.find("+coh_size=0x") == 0)
      coherent_size = strtoull(arg.substr(12).c_str(), 0, 16);

  }
}

// accesses to tohost/fromhost should use a coherent path
bool testchip_tsi_t::addr_should_be_coherent(addr_t taddr) {
  return taddr >= coherent_base && taddr < coherent_base + coherent_size;
}

void testchip_tsi_t::write_chunk(addr_t taddr, size_t nbytes, const void* src)
{
  // Loadmem should always use incoherent accesses
  addr_t addr = (!is_loadmem && addr_should_be_coherent(taddr)) ? taddr + coherent_offset : taddr;
  if (is_loadmem) {
    load_mem_write(addr, nbytes, src);
  } else {
    tsi_t::write_chunk(addr, nbytes, src);
  }
}

void testchip_tsi_t::read_chunk(addr_t taddr, size_t nbytes, void* dst)
{
  // Loadmem should always use incoherent accesses
  addr_t addr = (!is_loadmem && addr_should_be_coherent(taddr)) ? taddr + coherent_offset : taddr;
  if (is_loadmem) {
    load_mem_read(addr, nbytes, dst);
  } else {
    tsi_t::read_chunk(addr, nbytes, dst);
  }
}

void testchip_tsi_t::reset()
{
  for (auto p : init_accesses) {
    if (p.store) {
      fprintf(stderr, "Writing %lx with %x\n", p.address, p.stdata);
      write_chunk(p.address, sizeof(uint32_t), &p.stdata);
      fprintf(stderr, "Done writing %lx with %x\n", p.address, p.stdata);
    } else {
      fprintf(stderr, "Reading %lx ...", p.address);
      uint32_t rdata = 0;
      read_chunk(p.address, sizeof(uint32_t), &rdata);
      fprintf(stderr, " got %x\n", rdata);
    }
  }
  if (write_hart0_msip)
    tsi_t::reset();
}
