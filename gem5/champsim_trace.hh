 /*
 * Copyright (c) 2023 Texas A&M University
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file This file initializes a simple trace unit which listens to
 * a probe point in the fetch and commit stage of the O3 pipeline
 * and simply outputs those events as a dissassembled instruction stream
 * to the trace output.
 */
#ifndef __CPU_O3_PROBE_INST_TRACE_HH__
#define __CPU_O3_PROBE_INST_TRACE_HH__

#include "cpu/o3/dyn_inst_ptr.hh"
#include "params/ChampsimTrace.hh"
#include "sim/probe/probe.hh"

// Champsim parameters
namespace champsim
{
    constexpr std::size_t NUM_INSTR_DESTINATIONS = 2;
    constexpr std::size_t NUM_INSTR_SOURCES = 4;

    // special registers used by champsim
    constexpr char REG_STACK_POINTER = 6;
    constexpr char REG_FLAGS = 25;
    constexpr char REG_INSTRUCTION_POINTER = 26;
}

namespace gem5
{

namespace o3
{

class ChampsimTrace : public ProbeListenerObject
{

  public:
    ChampsimTrace(const ChampsimTraceParams &params) :
        ProbeListenerObject(params),
        champsim(params.champsim)
    {
    }

    /** Register the probe listeners. */
    void regProbeListeners() override;

    std::string
    name() const override
    {
        return ProbeListenerObject::name() + ".trace";
    }

  private:

    struct input_instr
    {
        // instruction pointer or PC (Program Counter)
        unsigned long long pc;

        // branch info
        unsigned char is_branch;
        unsigned char branch_taken;

        unsigned char destination_registers
            [champsim::NUM_INSTR_DESTINATIONS]; // output registers
        unsigned char source_registers
            [champsim::NUM_INSTR_SOURCES];      // input registers

        unsigned long long destination_memory
            [champsim::NUM_INSTR_DESTINATIONS]; // output memory
        unsigned long long source_memory
            [champsim::NUM_INSTR_SOURCES];      // input memory
    };

    std::ofstream outfile;

    input_instr champsim_instr = {};

    // Enable ChampSim trace generator
    bool champsim = false;

    void generateTrace(const DynInstConstPtr& dynInst);

    void traceBranch(const DynInstConstPtr& dynInst);
    void traceRegister(const DynInstConstPtr& dynInst);
    void traceMemory(const DynInstConstPtr& dynInst);

    void WriteCurrentInstruction() {
        typename decltype(outfile)::char_type buf[sizeof(input_instr)];
        std::memcpy(buf, &champsim_instr, sizeof(input_instr));
        outfile.write(buf, sizeof(input_instr));
    }

    template <typename T>
    void WriteToSet(T* begin, T* end, size_t length, T r) {
        T* set_end = std::find(begin, end, 0);
        // check to see if this register is already in the list
        T* found_reg = std::find(begin, set_end, r);
        if (found_reg < end)
            *found_reg = r;
    }
};

} // namespace o3
} // namespace gem5

#endif//__CPU_O3_PROBE_CHAMPSIM_TRACE_HH__
