library ieee ;
	use ieee.std_logic_1164.all ;
	use ieee.numeric_std.all ;

entity tb_sobel_core is
  
end tb_sobel_core ; 

architecture behavioral of tb_sobel_core is
	constant CLOCK_PERIOD 	: time := 10 ns;
	
	signal tb_rst : std_logic := '0';
	signal tb_clk : std_logic := '0';

	constant TB_RING_DEPTH 	: integer := 6;
	constant TB_RING_WIDTH 	: integer := 16;
	constant TB_PIXEL_SIZE		: integer := 8;
	constant TB_DATA_SIZE		: integer := 32;
	constant TB_IMAGE_V_SIZE	: integer := 16;
	constant TB_IMAGE_H_SIZE	: integer := 16;

	signal tb_aso_in0_data : std_logic_vector((9*8)-1 downto 0);	
	signal tb_aso_in0_ready : std_logic;
	signal tb_aso_in0_valid : std_logic;

	signal tb_aso_out0_data : std_logic_vector(8-1 downto 0);
	signal tb_aso_out0_ready : std_logic;
	signal tb_aso_out0_valid : std_logic;

	signal tb_empty : std_logic;
	signal tb_empty_next : std_logic;
	signal tb_full : std_logic;
	signal tb_full_next : std_logic;
	signal tb_fill_count : integer range TB_RING_DEPTH-1 downto 0;

	signal tb_counter : integer := 0;

	type ram_type is array (65535-1 downto 0) of std_logic_vector(72-1 downto 0);
	signal RAM: ram_type := (
		0 => X"A321C2A14D1D1563AD",
		1 => X"21E2F2C3D11A3D2E56",
		2 => X"486454186932165565",
		3 => X"A3541C6851B6861D68",
		4 => X"515A6151F85C15D564",
		5 => X"A448C69B684F685A46",
		6 => X"AE565F5E56C6DA1C5D",
		7 => X"A564C6D92A65E6A2F6",		
		others => (others =>'0')
	);

	component sobel_core is
		generic (
			RING_DEPTH 	: integer;	
		);
		port (
			clk: in std_logic;
			rst: in std_logic;

			aso_in0_data : in std_logic_vector((9*8)-1 downto 0);
			aso_in0_ready : out std_logic;
			aso_in0_valid : in std_logic;

			aso_out0_data 	: out 	std_logic_vector(8-1 downto 0);
			aso_out0_valid	: out 	std_logic;
			aso_out0_ready	: in 	std_logic;

			rxBuff_empty 			: out std_logic;
			rxBuff_empty_next 		: out std_logic;
			rxBuff_full 			: out std_logic;
			rxBuff_full_next 		: out std_logic;
			rxBuff_fill_count 		: out integer range RING_DEPTH-1 downto 0
  		) ;
  	end component sobel_core ; 


begin

	DUT: sobel_core
		generic map (
			RING_DEPTH 	=> TB_RING_DEPTH
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

			rxBuff_empty 		=> tb_empty,
			rxBuff_empty_next 	=> tb_empty_next,
			rxBuff_full 		=> tb_full,
			rxBuff_full_next 	=> tb_full_next,
			rxBuff_fill_count 	=> tb_fill_count
		);

	tb_clk <= not tb_clk after  CLOCK_PERIOD/2;

	counter: process(tb_rst, tb_clk)
	begin
		if tb_rst = '1' then
			tb_counter <= 0;
		elsif tb_clk'event and tb_clk = '1' then
			tb_counter <= tb_counter + 1;
		end if;
	end process counter;

	tb_aso_in0_data <= RAM(tb_counter);

	
	
	stim_proc : process
	begin 
		-- STARTING PROCEDURE 
		tb_aso_out0_ready <= '0';
		tb_aso_in0_valid <= '0';
		tb_rst <= '1';

		s_signed <= s_signed - to_signed(10, 16);
		wait for 10*CLOCK_PERIOD; -- 10 * 10 ns
		
		tb_rst <= '0';
		tb_aso_out0_ready <= '1';
		tb_aso_in0_valid <= '1';
		-- END OF STARTING PROCEDURE

		wait;
		
		assert false report "Simulation Ended! TEST PASSATO" severity failure;
	end process stim_proc;
		
end Behavioral;