library ieee ;
	use ieee.std_logic_1164.all ;
	use ieee.numeric_std.all ;

entity tb_tx_ringBuffer is

end tb_tx_ringBuffer ;

architecture behavioral of tb_tx_ringBuffer is
	constant CLOCK_PERIOD 		: time := 10 ns;

	constant TB_RING_DEPTH 		: integer := 6;
	constant TB_PIXEL_SIZE		: integer := 8;
	constant TB_DATA_SIZE		: integer := 32;
	constant TB_IMAGE_H_SIZE	: integer := 10;
	constant TB_IMAGE_V_SIZE	: integer := 10;
	constant TB_RAM_LENGTH		: integer := TB_IMAGE_H_SIZE * TB_IMAGE_V_SIZE;

	signal tb_rst : std_logic := '0';
	signal tb_clk : std_logic := '0';

	signal tb_aso_in0_data : std_logic_vector(TB_PIXEL_SIZE-1 downto 0);
	signal tb_aso_in0_ready : std_logic;
	signal tb_aso_in0_valid : std_logic;

	signal tb_aso_out0_data : std_logic_vector(TB_DATA_SIZE-1 downto 0);
	signal tb_aso_out0_ready : std_logic;
	signal tb_aso_out0_valid : std_logic;
	signal tb_aso_out0_sop	: std_logic;
	signal tb_aso_out0_eop	: std_logic;

	type input_state_type is (TRAMSISSION, NOTREADY);
	signal tb_input_state: input_state_type := NOTREADY;

	signal tb_pkt_cnt : integer := 0;
	signal tb_pkt_cnt_en : std_logic;
	signal tb_pkt_cnt_reset : std_logic;

	type ram_type is array (TB_RAM_LENGTH-1 downto 0) of std_logic_vector(TB_PIXEL_SIZE-1 downto 0);
	signal RAM: ram_type := (	0 	=> X"AA",	1 	=> X"A1", 	2 	=> X"A2", 	3 	=> X"A3", 	4 	=> X"A4", 	5 	=> X"A5", 	6 	=> X"A6", 	7 	=> X"A7", 	8 	=> X"A8", 	9 	=> X"A9",
								10 	=> X"AA",	11 	=> X"A1", 	12 	=> X"A2", 	13 	=> X"A3", 	14 	=> X"A4", 	15 	=> X"A5", 	16 	=> X"A6", 	17 	=> X"A7", 	18 	=> X"A8", 	19 	=> X"A9",
								20 	=> X"AA",	21 	=> X"A1", 	22 	=> X"A2", 	23 	=> X"A3", 	24 	=> X"A4", 	25 	=> X"A5", 	26 	=> X"A6", 	27 	=> X"A7", 	28 	=> X"A8", 	29 	=> X"A9",
								30 	=> X"AA",	31 	=> X"A1", 	32 	=> X"A2", 	33 	=> X"A3", 	34 	=> X"A4", 	35 	=> X"A5", 	36 	=> X"A6", 	37 	=> X"A7", 	38 	=> X"A8", 	39 	=> X"A9",
								40 	=> X"AA",	41 	=> X"A1", 	42 	=> X"A2", 	43 	=> X"A3", 	44 	=> X"A4", 	45 	=> X"A5", 	46 	=> X"A6", 	47 	=> X"A7", 	48 	=> X"A8", 	49 	=> X"A9",
								50 	=> X"AA",	51 	=> X"A1", 	52 	=> X"A2", 	53 	=> X"A3", 	54 	=> X"A4", 	55 	=> X"A5", 	56 	=> X"A6", 	57 	=> X"A7", 	58 	=> X"A8", 	59 	=> X"A9",
								60 	=> X"AA",	61 	=> X"A1", 	62 	=> X"A2", 	63 	=> X"A3", 	64 	=> X"A4", 	65 	=> X"A5", 	66 	=> X"A6", 	67 	=> X"A7", 	68 	=> X"A8", 	69 	=> X"A9",
								70 	=> X"AA",	71 	=> X"A1", 	72 	=> X"A2", 	73 	=> X"A3", 	74 	=> X"A4", 	75 	=> X"A5", 	76 	=> X"A6", 	77 	=> X"A7", 	78 	=> X"A8", 	79 	=> X"A9",
								80 	=> X"AA",	81 	=> X"A1", 	82 	=> X"A2", 	83 	=> X"A3", 	84 	=> X"A4", 	85 	=> X"A5", 	86 	=> X"A6", 	87 	=> X"A7", 	88 	=> X"A8", 	89 	=> X"A9",
								90 	=> X"AA",	91 	=> X"A1", 	92 	=> X"A2", 	93 	=> X"A3", 	94 	=> X"A4", 	95 	=> X"A5", 	96 	=> X"A6", 	97 	=> X"A7", 	98 	=> X"A8", 	99 	=> X"A9"
							);

	component tx_ringBuffer is
		generic (
            PIXEL_SIZE: integer := 8;
            DATA_SIZE: integer := 32;
            PACKET_SIZE: integer := 5754
		);

		port (
			clk		: in std_logic;
			rst	: in std_logic;

			-- Avalon Input Streaming data port
			aso_in0_data   : in  	std_logic_vector(PIXEL_SIZE-1 downto 0);		--  in0.data
			aso_in0_ready  : out 	std_logic; 					   				--     .ready
			aso_in0_valid  : in  	std_logic;                         			--     .valid

			-- Avalon Output Streaming data port
			aso_out0_data   : out  	std_logic_vector(DATA_SIZE-1 downto 0);			--  out0.data
			aso_out0_ready  : in 	std_logic; 					   				--  	.ready
			aso_out0_valid  : out 	std_logic;                         			--  	.valid
			aso_out0_sop	: out	std_logic;									--		.sop
			aso_out0_eop	: out	std_logic									--		.eop

 	);
	end component tx_ringBuffer;


begin

	DUT: tx_ringBuffer
		generic map (
			PIXEL_SIZE 	=> 8,
			DATA_SIZE	=> 32,
			PACKET_SIZE	=> 8
		)
		port map (
			clk	=> tb_clk,
			rst	=> tb_rst,

			aso_in0_data 	=> tb_aso_in0_data,
			aso_in0_ready 	=> tb_aso_in0_ready,
			aso_in0_valid 	=> tb_aso_in0_valid,

			aso_out0_data 	=> tb_aso_out0_data,
			aso_out0_ready 	=> tb_aso_out0_ready,
			aso_out0_valid 	=> tb_aso_out0_valid,
			aso_out0_sop	=> tb_aso_out0_sop,
			aso_out0_eop	=> tb_aso_out0_eop
		);


	tb_aso_in0_data <= RAM(tb_pkt_cnt mod 100);

	tb_clk <= not tb_clk after  CLOCK_PERIOD/2;

	tb_pktCounter: process(tb_clk, tb_rst)
	begin
		if tb_rst = '1' then
			tb_pkt_cnt <= 0;
		elsif tb_clk'event and tb_clk = '1' then
			if tb_pkt_cnt_reset = '1' then
				tb_pkt_cnt <= 0;
			elsif tb_pkt_cnt_en = '1' and tb_aso_in0_ready = '1' then
				tb_pkt_cnt <= tb_pkt_cnt + 1;
			end if;
		end if;
	end process tb_pktCounter;


	input_onStateChange: process(tb_input_state)
	begin
		tb_aso_in0_valid <= '0';
		tb_pkt_cnt_en <= '0';
		tb_pkt_cnt_reset <= '0';

		case tb_input_state is
			when TRAMSISSION =>
				tb_aso_in0_valid <= '1';
				tb_pkt_cnt_en <= '1';
			when NOTREADY =>
		end case;
	end process input_onStateChange;

	input_changeState: process(tb_clk, tb_rst)
	begin
		if tb_rst = '1' then
			tb_input_state <= TRAMSISSION;
		elsif tb_clk'event and tb_clk = '1' then
			case tb_input_state is
				when TRAMSISSION =>
					if tb_aso_in0_ready = '1' then
						if tb_pkt_cnt >= (TB_IMAGE_V_SIZE * TB_IMAGE_H_SIZE) - 1 then
							tb_input_state <= NOTREADY;
						end if;
					end if;
				when NOTREADY =>
			end case;
		end if;
	end process input_changeState;

	stim_proc : process
	begin
		-- STARTING PROCEDURE
		wait for 100 ns;
		wait for CLOCK_PERIOD;
		tb_aso_out0_ready <= '0';
		tb_rst <= '1';
		wait for CLOCK_PERIOD;
		tb_rst <= '0';

		tb_aso_out0_ready <= '1';
		wait for CLOCK_PERIOD;
		wait for CLOCK_PERIOD;
		wait for CLOCK_PERIOD;
		wait for CLOCK_PERIOD;
		tb_aso_out0_ready <= '0';
		wait for CLOCK_PERIOD;
        tb_aso_out0_ready <= '1';
		-- END OF STARTING PROCEDURE

		wait;

		assert false report "Simulation Ended! TEST PASSATO" severity failure;
	end process stim_proc;

end Behavioral;
