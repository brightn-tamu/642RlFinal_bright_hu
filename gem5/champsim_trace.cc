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

#include "cpu/o3/probe/champsim_trace.hh"

#include "base/trace.hh"
#include "cpu/o3/dyn_inst.hh"
#include "debug/ChampsimTrace.hh"

namespace gem5
{

namespace o3
{

void
ChampsimTrace::generateTrace(const DynInstConstPtr& dynInst)
{
    if (champsim) {
        auto disassemble = dynInst->staticInst->disassemble(
            dynInst->pcState().instAddr());
        if (disassemble.substr(2, 13) == "Microcode_ROM")
            return;

        if (dynInst->isFirstMicroop()) {
            champsim_instr = {};
            champsim_instr.pc = dynInst->pcState().instAddr();
        }

        DPRINTFR(ChampsimTrace, "[%s]: Commit 0x%08x %s.\n", name(),
             dynInst->pcState().instAddr(),
             disassemble);

        traceRegister(dynInst);

        if (dynInst->isControl()) {
            traceBranch(dynInst);
        }
        if (dynInst->isMemRef()) {
            traceMemory(dynInst);
        }

        DPRINTFR(ChampsimTrace, "\n");

        if (dynInst->isLastMicroop()) {
            DPRINTFR(ChampsimTrace, "ChampSim pc: 0x%x\n",
                champsim_instr.pc);
            DPRINTFR(ChampsimTrace, "ChampSim is_branch: %d\n",
                champsim_instr.is_branch);
            DPRINTFR(ChampsimTrace, "ChampSim branch_taken: %d\n",
                champsim_instr.branch_taken);
            for (size_t i = 0; i < champsim::NUM_INSTR_DESTINATIONS; i++) {
                DPRINTFR(ChampsimTrace, "Champsim destReg[%d]: 0x%x.\n",
                i, champsim_instr.destination_registers[i]);
            }
            for (size_t i = 0; i < champsim::NUM_INSTR_SOURCES; i++) {
                DPRINTFR(ChampsimTrace, "Champsim srcReg[%d]: 0x%x.\n",
                i, champsim_instr.source_registers[i]);
            }

            for (size_t i = 0; i < champsim::NUM_INSTR_DESTINATIONS; i++) {
                DPRINTFR(ChampsimTrace, "Champsim destMemory[%d]: 0x%x.\n",
                i, champsim_instr.destination_memory[i]);
            }
            for (size_t i = 0; i < champsim::NUM_INSTR_SOURCES; i++) {
                DPRINTFR(ChampsimTrace, "Champsim srcMemory[%d]: 0x%x.\n",
                i, champsim_instr.source_memory[i]);
            }
            WriteCurrentInstruction();

            DPRINTFR(ChampsimTrace, "\n\nNew instruction.\n");
        }
    }

}

void
ChampsimTrace::regProbeListeners()
{
    typedef ProbeListenerArg<ChampsimTrace,
            DynInstConstPtr> DynInstListener;
    outfile.open("champsim.trace",
        std::ios_base::binary | std::ios_base::trunc);
    if (!outfile) {
        std::cout << "Couldn't open output trace file. Exiting." << std::endl;
        exit(1);
    }

    listeners.push_back(new DynInstListener(this, "Commit",
                &ChampsimTrace::generateTrace));
}

void
ChampsimTrace::traceBranch(const DynInstConstPtr& dynInst)
{
    uint64_t pc_addr = dynInst->pcState().instAddr();
    uint64_t target = dynInst->branchTarget()->instAddr();
    // auto conditional = dynInst->isCondCtrl();
    // auto direct = dynInst->isDirectCtrl();
    std::string type = "";
    int64_t distance = int64_t(pc_addr - target);
    bool pred_taken = (distance > 4) ? true : false;
    // bool mispred = dynInst->mispredicted();
    bool actually_taken = dynInst->pcState().branching();

    if (dynInst->isCall()) {
        type = "call";
    } else if (dynInst->isReturn()) {
        type = "return";
    } else if (dynInst->isUncondCtrl() || dynInst->isCondCtrl()) {
        type = "jump";
    } else {
        type = "other";
    }

    DPRINTFR(ChampsimTrace, "[%s]: Branch | 0x%08x, "
        "%s, pred:%d, actual:%d, mispred:%d, dist:%i, target: 0x%08x "
        "| %s.\n", name(),
        pc_addr, type,
        pred_taken, actually_taken, pred_taken != actually_taken,
        distance, target,
        dynInst->staticInst->disassemble(pc_addr));

    champsim_instr.is_branch = 1;
    if (actually_taken)
        champsim_instr.branch_taken = 1;
}

void
ChampsimTrace::traceRegister(const DynInstConstPtr& dynInst)
{
    auto microop = dynInst->staticInst->getName();
    // srcReg
    if (microop == "rdip") {
        WriteToSet<unsigned char>(champsim_instr.source_registers,
                champsim_instr.source_registers + champsim::NUM_INSTR_SOURCES,
                champsim::NUM_INSTR_SOURCES,
                champsim::REG_INSTRUCTION_POINTER);
    }

    for (size_t i = 0; i < dynInst->numSrcRegs(); i++) {
        auto className = dynInst->srcRegIdx(i).className();
        unsigned char regNum = dynInst->srcRegIdx(i).index();

        if (className != nullptr) {
            if (regNum == champsim::REG_STACK_POINTER
                || regNum == champsim::REG_FLAGS
                || regNum == champsim::REG_INSTRUCTION_POINTER) {
                regNum = 70; // Random value
            }

            if (!strcmp(className, "CCRegClass")) {
                regNum = champsim::REG_FLAGS;
            }

            if (!strcmp(className, "IntRegClass")) {
                if (regNum == 4) {
                    regNum = champsim::REG_STACK_POINTER;
                }
            }

            DPRINTFR(ChampsimTrace, "[%s]: srcReg %d: id %d, class: %s.\n",
                name(), i, regNum, className);

            WriteToSet<unsigned char>(champsim_instr.source_registers,
                champsim_instr.source_registers + champsim::NUM_INSTR_SOURCES,
                champsim::NUM_INSTR_SOURCES,
                regNum);
        }
    }

    //destReg
    if (microop == "wrip" || microop == "wripi") {
        WriteToSet<unsigned char>(champsim_instr.destination_registers,
                champsim_instr.destination_registers +
                    champsim::NUM_INSTR_DESTINATIONS,
                champsim::NUM_INSTR_DESTINATIONS,
                champsim::REG_INSTRUCTION_POINTER);
    }

    for (size_t i = 0; i < dynInst->numDestRegs(); i++) {
        auto className = dynInst->destRegIdx(i).className();
        unsigned char regNum = dynInst->destRegIdx(i).index();

        if (className != nullptr) {
            if (!strcmp(className, "CCRegClass")) {
                regNum = champsim::REG_FLAGS;
            }

            DPRINTFR(ChampsimTrace, "[%s]: destReg %d: id %d, class: %s.\n",
            name(), i, regNum, className);

            WriteToSet<unsigned char>(champsim_instr.destination_registers,
                champsim_instr.destination_registers +
                champsim::NUM_INSTR_DESTINATIONS,
                champsim::NUM_INSTR_DESTINATIONS,
                regNum);
        }
    }
}

void
ChampsimTrace::traceMemory(const DynInstConstPtr& dynInst)
{
    unsigned long long pc_addr = dynInst->pcState().instAddr();
    unsigned long long addr = dynInst->effAddr;
    unsigned long long paddr = dynInst->physEffAddr;
    std::string type = dynInst->isLoad() ? "ld" : "st";

    DPRINTFR(ChampsimTrace,
        "[%s]: Mem | 0x%08x %s -> 0x%08x (0x%08x). | %s.\n",
        name(), pc_addr, type, addr, paddr,
        dynInst->staticInst->disassemble(pc_addr));

    if (dynInst->isLoad()) {
        WriteToSet<unsigned long long int>(champsim_instr.source_memory,
            champsim_instr.source_memory + champsim::NUM_INSTR_SOURCES,
            champsim::NUM_INSTR_SOURCES,
            addr);
    } else {
        WriteToSet<unsigned long long int>(champsim_instr.destination_memory,
            champsim_instr.destination_memory +
            champsim::NUM_INSTR_DESTINATIONS,
            champsim::NUM_INSTR_DESTINATIONS,
            addr);
    }
}


} // namespace o3
} // namespace gem5
