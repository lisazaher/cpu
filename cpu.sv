module cpu(input clk, reset, input [15:0] i_mem_rddata, output logic o_mem_rd, o_mem_wr, output logic [15:0] o_mem_addr, o_mem_wrdata);

	//signals
	logic ld_pc, pc_inc, ldpc7, Ain, Gin, AddSub, ldnz, ld_ir, ld_addr, ld_data, done;
	logic Gout, pcout, out8, s8out, s11out, wr, n, z;
	logic [15:0] buswire, wA, wG, wPC, wr0, wr1, wr2, wr3, wr4, wr5, wr6, wr7, sext_imm8, sext_imm11, IR, addr, data, sum;
	logic [7:0] Xreg, Yreg, Rin, Rout;
	logic [12:0] Sel;
	logic [7:0] imm8;
	logic [10:0] imm11;
	logic [4:0] inst;

	//instructions
	enum int unsigned
	{
		INIT,
		T0,
		T1,
		T2,
		T3,
		T4,
		T5,
		T6
	} state, nextstate;

	/*enum bit [4:0] {
		mv, add, sub, cmp, ld, st, mvi, addi, subi, cmpi, mvhi, jr, jzr, jnr, callr, j,
		jz, jn, call
	} inst;*/

	// Control FSM flip-flops
	always @(posedge clk)
		if (reset)
			state <= INIT;
		else
			state <= nextstate;	

	// Control FSM state table
	always @(state, done)
	begin
		case (state)
			INIT:
				nextstate = T0;
			T0: // instruction fetch
				nextstate = T1;
			T1: begin
 				if (done) nextstate = T0;
 				else nextstate = T2;
 			end
			T2: 
 			begin
 				if (done) nextstate = T0;
				else nextstate = T3;
			end
	 		T3: // some instructions end after this time step
	 		begin
				if (done) nextstate = T0;
 				else nextstate = T4;
 			end
			T4: //same ehre
			begin
 				if (done) nextstate = T0;
 				else nextstate = T5;
 			end
			T5: // instructions end after this time step
			begin
 				if (done) nextstate = T0;
 				else nextstate = T6;
 			end
 			T6: nextstate = T0;

		endcase
	end

	// Control FSM outputs
	always @(*)
	begin
		parameter 
		mv = 5'b0, add = 5'b1, sub = 5'b10, cmp = 5'b11, ld = 5'b100, st = 5'b101, 
		mvi = 5'b10000, addi = 5'b10001, subi = 5'b10010, cmpi = 5'b10011, mvhi = 5'b10110,
		jr = 5'b1000, jzr = 5'b1001, jnr = 5'b1010, callr = 5'b1100, 
		j = 5'b11000, jz = 5'b11001, jn = 5'b11010, call = 5'b11100;

		ld_pc = 0; pc_inc = 0; ldpc7 = 0; out8 = 0; Ain = 0; Gin = 0; AddSub = 0; ldnz = 0; 
		ld_ir = 0; ld_addr = 0; ld_data = 0; done = 0;
		Gout = 0; pcout = 0; s8out = 0; s11out = 0;
		o_mem_rd = 0; o_mem_wr = 0; wr = 0;
		Rin = 0;
		Rout = 0;
		case (state)
			INIT: begin
				ld_addr = 1;
				o_mem_rd = 1;
			end
			T0: // fetch the instruction
				begin
					ld_ir = 1;
					pc_inc = 1; // to increment pc
				end
			T1:
				case (inst)
					mv:
					begin
						Rout = Yreg;
						Rin = Xreg;
						ld_addr = 1;
					end
					add, sub, cmp: begin
						Rout = Xreg;
						Ain = 1;
					end
					ld, st: begin
						Rout = Yreg;
						wr = 1;
						ld_addr = 1;
					end
					mvi: begin
						s8out = 1;
						Rin = Xreg;
						ld_addr = 1;
					end
					addi, subi, cmpi: begin
						Rout = Xreg;
						Ain = 1;
					end
					mvhi: begin
						out8 = 1;
						Rin = Xreg;
						ld_addr = 1;
					end
					jr: begin
						Rout = Xreg;
						ld_pc = 1;
					end
					jzr: begin
						if (z) begin
							Rout = Xreg;
							ld_pc = 1;
						end
						else begin
							ld_addr = 1;
						end
					end
					jnr: begin
						if (n) begin
							Rout = Xreg;
							ld_pc = 1;
						end
						else begin
							ld_addr = 1;
						end					
					end
					callr: begin
						pcout = 1;
						Rin = 8'b10000000;
						ldpc7 = 1;
					end
					j: begin
						s11out = 1;
						Ain = 1;
					end
					jz: begin
						if (z) begin
							s11out = 1;
							Ain = 1;
						end
						else begin
							ld_addr = 1;
						end
					end
					jn: begin
						if (n) begin
							s11out = 1;
							Ain = 1;
						end
						else begin
							ld_addr = 1;
						end					
					end
					call: begin
						pcout = 1;
						Rin = 8'b10000000;
						ldpc7 = 1;
					end
					default: ;
			   endcase
			T2:
				case (inst)
					mv, mvi, mvhi:
					begin
						done = 1;
						o_mem_rd = 1;
					end
					add: begin
						Rout = Yreg;
						Gin = 1;
					end
					sub, cmp: begin
						Rout = Yreg;
						Gin = 1;
						AddSub = 1;
					end
					ld: begin
						o_mem_rd = 1;
					end
					st: begin
						Rout = Xreg;
						wr = 1;
						ld_data = 1;
					end
					addi: begin
						s8out = 1;
						Gin = 1;
					end
					subi, cmpi: begin
						s8out = 1;
						Gin = 1;
						AddSub = 1;	
					end
					jr: begin
						ld_addr = 1;
					end
					jzr: begin
						if (z) begin
							ld_addr = 1;
						end
						else begin
							done = 1;
							o_mem_rd = 1;
						end
					end
					jnr: begin
						if (n) begin
							ld_addr = 1;
						end
						else begin
							done = 1;
							o_mem_rd = 1;
						end					
					end
					callr: begin
						Rout = Xreg;
						ld_pc = 1;
					end
					j: begin
						pcout = 1;
						Gin = 1;
					end
					jz: begin
						if (z) begin
							pcout = 1;
							Gin = 1;
						end
						else begin
							done = 1;
							o_mem_rd = 1;
						end
					end
					jn: begin
						if (n) begin
							pcout = 1;
							Gin = 1;
						end
						else begin
							done = 1;
							o_mem_rd = 1;
						end					
					end
					call: begin
						s11out = 1;
						Ain = 1;
					end
					default: ;
			   endcase
			T3:
				case (inst)
					add, sub: begin
						Gout = 1;
						Rin = Xreg;
						ldnz = 1;
						ld_addr = 1;
					end
					cmp: begin
						Gout = 1;
						ldnz = 1;
						ld_addr = 1;
					end
					ld: begin
						Rout = 0;
						Rin = Xreg;
						ld_addr = 1;
					end
					st: begin
						o_mem_wr = 1;
					end
					addi, subi: begin
						Gout = 1;
						Rin = Xreg;
						ldnz = 1;
						ld_addr = 1;
					end
					cmpi: begin
						Gout = 1;
						ldnz = 1;
						ld_addr = 1;
					end
					jr, jzr, jnr: begin
						done = 1;
						o_mem_rd = 1;
					end
					callr: begin
						ld_addr = 1;
					end
					j, jz, jn: begin
						Gout = 1;
						ld_pc = 1;
					end
					call: begin
						pcout = 1;
						Gin = 1;
					end
					default: ;
			   endcase
			T4:
				case (inst)
					add, sub, cmp, ld, addi, subi, cmpi, callr: begin
						done = 1;
						o_mem_rd = 1;
					end
					cmp: begin
						Gout = 1;
						ldnz = 1;
						ld_addr = 1;
					end
					st: begin
						ld_addr = 1;
					end
					j, jz, jn: begin
						ld_addr = 1;
					end
					call: begin
						Gout = 1;
						ld_pc = 1;
					end
					default: ;
			   endcase
			T5:
				case (inst)
					st, j, jz, jn: begin
						done = 1;
						o_mem_rd = 1;
					end
					call: begin
						ld_addr = 1;
					end
					default: ;
			   endcase
			T6:
				case (inst)
					call: begin
						done = 1;
						o_mem_rd = 1;
					end
					default: ;
			   endcase
			default: ;
		endcase
	end

	//set up registers
	// 1st - IR
	assign inst = IR[4:0];
	assign imm8 = IR[15:8];
	assign imm11 = IR[15:5];
	dec3to8 decX (IR[7:5], 1'b1, Xreg);
	dec3to8 decY (IR[10:8], 1'b1, Yreg);
	regn IRreg (i_mem_rddata, ld_ir, clk, IR);	

	//2nd general purpose registers
	genpurp r0 (buswire, Rin[0], out8, 1'b0, clk, reset, wr0);
	genpurp r1 (buswire, Rin[1], out8, 1'b0, clk, reset, wr1);
	genpurp r2 (buswire, Rin[2], out8, 1'b0, clk, reset, wr2);
	genpurp r3 (buswire, Rin[3], out8, 1'b0, clk, reset, wr3);
	genpurp r4 (buswire, Rin[4], out8, 1'b0, clk, reset, wr4);
	genpurp r5 (buswire, Rin[5], out8, 1'b0, clk, reset, wr5);
	genpurp r6 (buswire, Rin[6], out8, 1'b0, clk, reset, wr6);
	genpurp r7 (buswire, Rin[7], out8, ldpc7, clk, reset, wr7); //only one that act gets ldpc signal

	//3rd PC
	pc_count pc(buswire, reset, clk, ld_pc, pc_inc, wPC);

	//4th - other A, G, addr, data
	regn rA (buswire, Ain, clk, wA);
	regn rG (sum, Gin, clk, wG);
	regn raddr (addr, ld_addr, clk, o_mem_addr);
	regn rdata (data, ld_data, clk, o_mem_wrdata);

	//5th - smol registers n and z
	always @(posedge clk) begin
	 	if (reset) begin
	 		n = 0;
	 		z = 0;
	 	end
	 	if (ldnz) begin
	 		n = buswire[15];
	 		z = (buswire == 16'b0) ? 1 : 0;
	 	end
	end

	//	alu
	always @(AddSub or wA or buswire)
		begin
		if (!AddSub)
			sum = wA + buswire;
	    else
			sum = wA - buswire;
		end

	//define mux for address and data
	assign addr = (wr) ? buswire : wPC;
	assign data = (wr) ? buswire : 16'b0;

	// define the internal processor bus
	assign Sel = {Rout, Gout, pcout, out8, s8out, s11out};

	always @(*)
	begin
		if (Sel == 13'b1000000000000)
			buswire = wr7;
   	else if (Sel == 13'b100000000000)
			buswire = wr6;
		else if (Sel == 13'b10000000000)
			buswire = wr5;
		else if (Sel == 13'b1000000000)
			buswire = wr4;
		else if (Sel == 13'b100000000)
			buswire = wr3;
		else if (Sel == 10'b10000000)
			buswire = wr2;
		else if (Sel == 10'b1000000)
			buswire = wr1;
		else if (Sel == 10'b100000)
			buswire = wr0;
		else if (Sel == 10'b10000)
			buswire = wG;
		else if (Sel == 10'b1000)
			buswire = wPC;
		else if (Sel == 10'b100)
			buswire = {imm8, 8'b0}; //move to msb of bus wires
		else if (Sel == 10'b10)
			buswire = {{8{imm8[7]}}, imm8}; //sext
		else if (Sel == 10'b1)
			buswire = 2*{{4{imm11[10]}}, imm11}; //sext (and *2)
   		else buswire = i_mem_rddata;
	end


endmodule

module dec3to8(input [2:0] in, input enable, output logic [7:0] Y);
	always @(in or enable)
	begin
		if (enable)
			case (in)
				3'b000: Y = 8'b1;
	   	   		3'b001: Y = 8'b10;
				3'b010: Y = 8'b100;
				3'b011: Y = 8'b1000;
				3'b100: Y = 8'b10000;
				3'b101: Y = 8'b100000;
				3'b110: Y = 8'b1000000;
				3'b111: Y = 8'b10000000;
			endcase
		else 
			Y = 8'b0;
	end
endmodule

module genpurp(input [15:0] R, input Rin, out8, ldpc7, clk, reset, output logic [15:0] Q);
	always @(posedge clk)
	 	if (reset)
	 		Q <= 16'b0;
	 	else if (out8 & Rin)
			Q <= Q + R;
		else if (Rin) 
			Q <= R; //r is imm8 shifted left (padded w/ 0s)
		else if (ldpc7)
			Q <= R; //only for r7
endmodule

module pc_count(input [15:0] R, input reset, Clock, ld_pc, pc_inc, output logic [15:0] Q);
	always @(posedge Clock)
	 	if (reset)
			Q <= 16'b0;
		else if (ld_pc)
			Q <= R;
		else if (pc_inc)
			Q <= Q + 2'b10; //+2
endmodule

module regn(R, Rin, Clock, Q);
	parameter n = 16;
	input [n-1:0] R;
	input Rin, Clock;
	output [n-1:0] Q;
	reg [n-1:0] Q;

	always @(posedge Clock)
	 	if (Rin)
			Q <= R;
endmodule

