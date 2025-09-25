# <center>RV32I Single-Cycle CPU</center>

<p align="center">
A Verilog implementation of a <b>single-cycle RISC-V RV32I processor</b> supporting ALU, memory, and control instructions.
</p>

<hr>

<h2>Features</h2>
<ul>
  <li><b>Arithmetic Instructions:</b> ADD, SUB, AND, OR, XOR, SLL, SRL, SRA, SLT, SLTU</li>
  <li><b>Immediate Instructions:</b> ADDI, ANDI, ORI, XORI, SLLI, SRLI, SRAI, SLTI, SLTIU</li>
  <li><b>Memory Instructions:</b> LW, LH, LHU, LB, LBU, SW, SH, SB</li>
  <li><b>Control Instructions:</b> BEQ, BNE, BLT, BGE, BLTU, BGEU, JAL, JALR</li>
  <li><b>Upper Immediate:</b> LUI, AUIPC</li>
</ul>

<h2>Modules</h2>
<table>
  <tr><th>Module</th><th>Description</th></tr>
  <tr><td>rv32i_top.v</td><td>Top-level module</td></tr>
  <tr><td>datapath.v</td><td>Datapath connections</td></tr>
  <tr><td>control.v</td><td>Control logic for instructions</td></tr>
  <tr><td>alu.v</td><td>Arithmetic and logic operations</td></tr>
  <tr><td>rfile.v</td><td>Register file (32 registers)</td></tr>
  <tr><td>imem.v</td><td>Instruction memory</td></tr>
  <tr><td>dmem.v</td><td>Data memory with byte enables</td></tr>
  <tr><td>extend.v</td><td>Immediate extension for all RV32I types</td></tr>
  <tr><td>mux2.v / mux4.v</td><td>2-to-1 / 4-to-1 multiplexers</td></tr>
  <tr><td>flopr.v</td><td>Program counter register</td></tr>
  <tr><td>adder.v</td><td>PC & branch adders</td></tr>
</table>

<h2>Directory Structure</h2>
<pre>
rv32i_single_cycle/
├── src/             # Verilog modules
├── testbench/       # Assembly tests & testbench
├── imem.mem         # Instruction memory
├── dmem.mem         # Data memory
└── README.md
</pre>

<h3>Run Testbench</h3>
<p>Verilog simulator (Verilator, XSIM, ModelSim), RISC-V assembler (Venus or local toolchain)</p>

<h2>Usage</h2>
<ul>
  <li>Replace <code>imem.mem</code> with compiled assembly instructions</li>
  <li>Replace <code>dmem.mem</code> with initial data if needed</li>
  <li>Test ALU, memory, and branching via Verilog testbenches</li>
</ul>

<h2>License</h2>
<p>Released under the MIT License.</p>

<h2>Author</h2>
<p><b>Yusuf Tekin</b> – Electronics & Communication Engineer, tekiny20@itu.edu.tr.</p>
