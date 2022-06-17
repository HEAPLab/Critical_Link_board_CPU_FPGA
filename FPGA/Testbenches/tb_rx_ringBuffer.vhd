library ieee ;
	use ieee.std_logic_1164.all ;
	use ieee.numeric_std.all ;

entity tb_rx_ringBuffer is
  
end tb_rx_ringBuffer ; 

architecture behavioral of tb_rx_ringBuffer is
	constant CLOCK_PERIOD 	: time := 10 ns;

	constant TB_RING_DEPTH 		: integer := 6;
	constant TB_PIXEL_SIZE		: integer := 8;
	constant TB_DATA_SIZE		: integer := 32;
	constant TB_IMAGE_H_SIZE	: integer := 12;
	constant TB_IMAGE_V_SIZE	: integer := 12;
	constant TB_RAM_LENGTH		: integer := TB_IMAGE_H_SIZE * TB_IMAGE_V_SIZE;
	
	signal tb_rst : std_logic := '0';
	signal tb_clk : std_logic := '0';

	signal tb_aso_in0_data : std_logic_vector(TB_DATA_SIZE-1 downto 0);	
	signal tb_aso_in0_eop : std_logic;
	signal tb_aso_in0_sop : std_logic;
	signal tb_aso_in0_ready : std_logic;
	signal tb_aso_in0_valid : std_logic;

	signal tb_aso_out0_data : std_logic_vector((9*8)-1 downto 0);
	signal tb_aso_out0_ready : std_logic;
	signal tb_aso_out0_valid : std_logic;

	signal tb_empty : std_logic;
	signal tb_empty_next : std_logic;
	signal tb_full : std_logic;
	signal tb_full_next : std_logic;
	signal tb_fill_count : integer range TB_RING_DEPTH-1 downto 0;

	type input_state_type is (TRAMSISSION, NOTREADY);
	signal tb_input_state: input_state_type := NOTREADY;

	signal tb_pkt_cnt : integer := 0;
	signal tb_pkt_cnt_en : std_logic;
	signal tb_pkt_cnt_reset : std_logic;

	type ram_type is array (65535-1 downto 0) of std_logic_vector(TB_DATA_SIZE-1 downto 0);
	signal RAM: ram_type := (
		0 => X"00010203",
		1 => X"04050607",
		2 => X"08090A0B",
		3 => X"0C0D0E0F",
		4 => X"10111213",
		5 => X"14151617",
		6 => X"18191A1B",
		7 => X"1C1D1E1F",		
		others => (others =>'0')
	);
	--signal RAM: ram_type := (
	--	(X"00", X"01", X"02", X"03", X"04", X"05", X"06", X"07", X"08", X"09", X"0A", X"0B", X"0C", X"0D", X"0E", X"0F"),
	--	(X"10", X"11", X"12", X"13", X"14", X"15", X"16", X"17", X"18", X"19", X"1A", X"1B", X"1C", X"1D", X"1E", X"1F"),
	--	(X"20", X"21", X"22", X"23", X"24", X"25", X"26", X"27", X"28", X"29", X"2A", X"2B", X"2C", X"2D", X"2E", X"2F"),
	--	(X"30", X"31", X"32", X"33", X"34", X"35", X"36", X"37", X"38", X"39", X"3A", X"3B", X"3C", X"3D", X"3E", X"3F"),
	--	(X"40", X"41", X"42", X"43", X"44", X"45", X"46", X"47", X"48", X"49", X"4A", X"4B", X"4C", X"4D", X"4E", X"4F"),
	--	(X"50", X"51", X"52", X"53", X"54", X"55", X"56", X"57", X"58", X"59", X"5A", X"5B", X"5C", X"5D", X"5E", X"5F"),
	--	(X"60", X"61", X"62", X"63", X"64", X"65", X"66", X"67", X"68", X"69", X"6A", X"6B", X"6C", X"6D", X"6E", X"6F"),
	--	(X"70", X"71", X"72", X"73", X"74", X"75", X"76", X"77", X"78", X"79", X"7A", X"7B", X"7C", X"7D", X"7E", X"7F"),
	--	(X"80", X"81", X"82", X"83", X"84", X"85", X"86", X"87", X"88", X"89", X"8A", X"8B", X"8C", X"8D", X"8E", X"8F")
	--)

	component rx_ringBuffer is 
		generic (
			RING_DEPTH 	: integer;		
			IMAGE_H_SIZE 	: integer;	
			IMAGE_v_SIZE	: integer;
			PIXEL_SIZE 	: integer;		-- bit
			DATA_SIZE	: integer 	-- bit
		);

		port (
			clk		: in std_logic;
			rst	: in std_logic;

			-- Avalon Input Streaming data port
			aso_in0_data   : in  	std_logic_vector(DATA_SIZE-1 downto 0);		--  in0.data
			aso_in0_ready  : out 	std_logic; 					   				--     .ready
			aso_in0_valid  : in  	std_logic;                         			--     .valid

			-- Avalon Output Streaming data port
			aso_out0_data   : out  	std_logic_vector((9*8)-1 downto 0);			--  out0.data
			aso_out0_ready  : in 	std_logic; 					   				--  	.ready
			aso_out0_valid  : out 	std_logic;                         			--  	.valid

			-- Flags
			empty 			: out std_logic;
			empty_next 		: out std_logic;
			full 			: out std_logic;
			full_next 		: out std_logic;

			-- The number of elements in the Ring Buffer
			fill_count 		: out integer range RING_DEPTH-1 downto 0

		);
	end component rx_ringBuffer;


begin

	DUT: rx_ringBuffer
		generic map (
			RING_DEPTH 	=> TB_RING_DEPTH,
			PIXEL_SIZE	=> TB_PIXEL_SIZE,
			DATA_SIZE	=> TB_DATA_SIZE,
			IMAGE_H_SIZE => TB_IMAGE_H_SIZE,
			IMAGE_v_SIZE => TB_IMAGE_v_SIZE
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

			empty 		=> tb_empty,
			empty_next 	=> tb_empty_next,
			full 		=> tb_full,
			full_next 	=> tb_full_next,
			fill_count 	=> tb_fill_count
		);


	tb_aso_out0_ready <= '1';
	
	tb_aso_in0_data <= RAM(tb_pkt_cnt);

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
						if tb_pkt_cnt >= ((TB_IMAGE_V_SIZE * TB_IMAGE_H_SIZE) / 4) - 1 then
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
		-- END OF STARTING PROCEDURE

		wait;
		
		assert false report "Simulation Ended! TEST PASSATO" severity failure;
	end process stim_proc;
		
end Behavioral;
