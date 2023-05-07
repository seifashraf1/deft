from capstone import Cs, CS_ARCH_X86, CS_MODE_64
from triton import ARCH_X86_64, CPUSIZE_QWORD, MemoryAccess, TritonContext

# Initialize Capstone and Triton
md = Cs(CS_ARCH_X86, CS_MODE_64)
triton = TritonContext(arch=ARCH_X86_64)

# Define a memory address to examine
addr = 0x1000

# Read the bytes of the instruction from memory
inst_bytes = b"\x48\x8B\x05\xB8\x13\x00\x00"  # Example instruction: mov rax, qword ptr [rip+0x13b8]

# Disassemble the instruction
inst = list(md.disasm(inst_bytes, addr))[0]

# Decode the instruction operands
operands = list(md.reg_name(inst.operands[i].reg) if inst.operands[i].type == 1 else inst.operands[i].mem for i in range(inst.op_count))

# Check for taint on the operands
for operand in operands:
    if triton.isMemoryTainted(operand):
        print(f"Taint found on operand {operand}")

# Propagate taint to the instruction destination
dst = inst.operands[0].mem
triton.taintMemory(MemoryAccess(dst, CPUSIZE_QWORD))

# Print the taint state as a bit vector
taint_state = triton.getTaintMemoryMap()
bit_vector = ''.join(str(int(taint_state[addr])) for addr in range(addr, addr + len(inst_bytes)))
print(f"Taint state as bit vector: {bit_vector}")
