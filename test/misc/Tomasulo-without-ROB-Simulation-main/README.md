# Tomasulo-without-ROB-Simulation
Issues:
-RAW hazards: we used maps for the register status and the register file. In an if condition, simply, we check if one of the source 
registers is in a currently executing instruction., in the status map, we wait until this value is finished, and then we start executing the'
instruction. After doing this for each instruction we write the results of the source register, we store the value in the rd and the register
file to avoid upcoming RAW hazards and for more program accuracy.

-PC: the PC is typically a counter that indicates which instruction is currently issuing; hence, we used the issue counter to represent the
 PC. This PC value is essential in the beq, jalr, and return instructions. In the beq instructions we wait until the branch result is computed, 
then we use the next PC which is the next instr. To execute to be the issue counter to represent jumping to the required instruction.


-Jalr address computation:  for a normal jalr instruction the return adress and the jump address are computed in the execute stage

Assumptions:

1. Fetching and decoding take 0 cycles and the instruction queue is already filled with all the instructions to be simulated.
2. No floating-point instructions, registers, or functional units
3. No input/output instructions are supported
4. No interrupts or exceptions are to be handled
5. For each program being executed, assume that the program and its data are fully loaded in the main memory
6. There is a one-to-one mapping between reservation stations and functional units. i.e., Each reservation station has a functional unit dedicated to it
7. No cache or virtual memory
8. Tomasulo's algorithm works
9. div has a dedicated resefvation station
10. subtract takes 2 clock cycles

What works:
- The in-order issuing
- the queue of the load and store

What doesn't work:
- in some testcases, an addi instruction did not take a correct write cycle
- the loop unrolling did not work properly
