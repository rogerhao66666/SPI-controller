///////////////////////////////////////////////////////////////////////////////
// Description: SPI (Serial Peripheral Interface) Controller (Master)
//
// Note:        i_clk must be at least 2x faster than spi_sclk
//
// Parameters:  SPI_MODE, can be 0, 1, 2, or 3.  See above.
//              Can be configured in one of 4 modes:
//              Mode | Clock Polarity (CPOL/CKP) | Clock Phase (CPHA)
//               0   |             0             |        0
//               1   |             0             |        1
//               2   |             1             |        0
//               3   |             1             |        1
//              CLK_PER_HALF_BIT - Sets frequency of spi_sclk.  spi_sclk is
//              derived from i_clk.  Set to integer number of clocks for each
//              half-bit of SPI data.  E.g. 100 MHz i_clk, CLK_PER_HALF_BIT = 2
//              would create spi_sclk of 25 MHz.  Must be >= 2
//
// Auther:		Roger
///////////////////////////////////////////////////////////////////////////////

module spi_controller
#(
	parameter SPI_MODE = 0,
	parameter DATA_BW = 8,
	parameter CLK_PER_HALF_BIT = 2
)
(
	input i_clk,
	input i_rstn,
	
	input i_tx_en,
	input [DATA_BW-1:0] i_tx_data,
	output o_tx_ready,
	
	output o_rx_ack,
	output [DATA_BW-1:0] o_rx_data,
	
	output spi_sclk,
	input spi_miso,
	output spi_mosi,
	output spi_cs
);

localparam CNT_BW = $clog2(DATA_BW);

localparam S_IDLE = 0;
localparam S_START = 1;
localparam S_SHIFT = 2;
localparam S_END = 3;

reg [3:0] state;

wire clk, rstn;
wire tx_en, tx_ready;
wire [DATA_BW-1:0] tx_data;
reg rx_ack;
reg cs;

wire CPOL, CPHA;


reg [$clog2(CLK_PER_HALF_BIT*2)-1:0] spi_clk_cnt;
reg spi_clk;
reg spi_clk_leading_edge;
reg spi_clk_trailing_edge;
reg [4:0] spi_clk_edge_cnt;
reg spi_clk_en;
reg spi_clk_cnt_reset;
reg o_spi_clk;

reg shift, load;
reg [CNT_BW-1:0] sft_cnt;
reg [DATA_BW:0] data_sft_reg;
reg [DATA_BW-1:0] data_mem;

//=======================================================================
// internal signal declaration
//=======================================================================

assign clk = i_clk;
assign rstn = i_rstn;
assign tx_en = i_tx_en;
assign tx_data = i_tx_data;
assign o_tx_ready = tx_ready;
assign o_rx_data = data_mem;
assign o_rx_ack = rx_ack;
assign spi_sclk = o_spi_clk;
assign spi_mosi = data_sft_reg[DATA_BW];
assign spi_cs = cs;

//-- SPI_MODE setting
assign CPOL = (SPI_MODE == 2) || (SPI_MODE == 3);
assign CPHA = (SPI_MODE == 1) || (SPI_MODE == 3);

assign tx_ready = (state[S_IDLE]) ? 1'b1 : 1'b0;

//=======================================================================
// RTL begin
//=======================================================================

//-- spi_sclk generation
always @(posedge clk or negedge rstn) begin
	if (~rstn) begin
		spi_clk_cnt <= 0;
		spi_clk <= CPOL;
		spi_clk_leading_edge <= 1'b0;
		spi_clk_trailing_edge <= 1'b0;
	end
	else begin
		spi_clk_leading_edge <= 1'b0;
		spi_clk_trailing_edge <= 1'b0;
		if (spi_clk_cnt_reset) begin
			spi_clk_edge_cnt <= 16;
			spi_clk_cnt <= 0;
		end
		else if (spi_clk_en) begin
			if (spi_clk_cnt == CLK_PER_HALF_BIT*2-1) begin
				spi_clk_edge_cnt <= spi_clk_edge_cnt - 1'b1;
				spi_clk_trailing_edge <= 1'b1;
				spi_clk_en <= 0;
				spi_clk <= ~spi_clk;
			end
			else if (spi_clk_cnt == CLK_PER_HALF_BIT-1) begin
				spi_clk_edge_cnt <= spi_clk_edge_cnt - 1'b1;
				spi_clk_leading_edge <= 1'b1;
				spi_clk_cnt <= spi_clk_cnt + 1'b1;
				spi_clk <= ~spi_clk;
			end
			else begin
				spi_clk_cnt <= spi_clk_cnt + 1'b1;
			end
		end
		else begin
			spi_clk <= CPOL;
		end
	end
end

//--FSM logic
always @(posedge clk or negedge rstn) begin
	if (~rstn) begin
		state <= 4'b0;
		state[S_IDLE] <= 1'b1;
		cs <= 1'b1;
		load <= 1'b0;
		shift <= 1'b0;
		spi_clk_cnt_reset <= 1'b0;
		spi_clk_en <= 1'b0;
	end
	else begin
		state <= 4'b0;
		case(1'b1)
			state[S_IDLE]: begin
				if (tx_en) begin
					state[S_START] <= 1'b1;
					spi_clk_cnt_reset <= 1'b1;
					load <= 1'b1;
					cs <= 1'b0;
				end
				else begin
					state[S_IDLE] <= 1'b1;
					cs <= 1'b1;
				end
			end
			state[S_START]: begin
				state[S_SHIFT] <= 1'b1;
				load <= 1'b0;
				shift <= 1'b1;
				spi_clk_en <= 1'b1;
				spi_clk_cnt_reset <= 1'b0;
			end
			state[S_SHIFT]: begin
				if (spi_clk_edge_cnt == 0) begin
					state[S_END] <= 1'b1;
					shift <= 1'b0;
					spi_clk_en <= 1'b0;
				end
				else begin
					state[S_SHIFT] <= 1'b1;
				end
			end
			state[S_END]: begin
				cs <= 1'b1;
				state[S_IDLE] <= 1'b1;
			end
			default: begin
				state <= 4'b0;
				state[S_IDLE] <= 1'b1;
				cs <= 1'b1;
				load <= 1'b0;
				shift <= 1'b0;
				spi_clk_cnt_reset <= 1'b0;
				spi_clk_en <= 1'b0;
			end
		endcase
	end
end

//--Generate MOSI data
always @(posedge clk or negedge rstn) begin
	if (~rstn) begin
		data_sft_reg <= {(DATA_BW+1){1'b0}};
	end
	else if (load) begin
		if (~CPHA) begin
			data_sft_reg[1+:DATA_BW] <= tx_data;
		end
		else begin
			data_sft_reg[0+:DATA_BW] <= tx_data;
		end
	end
	else if (shift) begin
		if ((spi_clk_leading_edge & CPHA) | (spi_clk_trailing_edge & ~CPHA)) begin
			data_sft_reg <= {data_sft_reg[0+:DATA_BW], 1'b0};
		end
	end
end

//--Read in MISO data
always @(posedge clk or negedge rstn) begin
	if (~rstn) begin
		data_mem <= {DATA_BW{1'b0}};
		sft_cnt <= {CNT_BW{1'b0}};
		rx_ack <= 1'b0;
	end
	else if (load) begin
		sft_cnt <= DATA_BW-1;
	end
	else if (shift) begin
		if ((spi_clk_leading_edge & ~CPHA) | (spi_clk_trailing_edge & CPHA)) begin
			data_mem <= {data_mem[0+:(DATA_BW-1)], spi_miso};
			sft_cnt <= sft_cnt - 1'b1;
			if (sft_cnt == 0) begin
				rx_ack <= 1'b1;
			end
			else begin
				rx_ack <= 1'b0;
			end
		end
		else begin
			rx_ack <= 1'b0;
		end
	end
	else begin
		rx_ack <= 1'b0;
	end
end

//--Add clock delay to signals for alignment
always @(posedge clk or negedge rstn) begin
	if (~rstn) begin
		o_spi_clk <= 1'b0;
	end
	else begin
		o_spi_clk <= spi_clk;
	end
end

endmodule