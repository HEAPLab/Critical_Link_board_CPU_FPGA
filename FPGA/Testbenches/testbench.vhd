library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use STD.textio.all;
use ieee.std_logic_textio.all;
USE ieee.numeric_bit.ALL;
 use ieee.std_logic_arith.all;
 
entity testbench is
  
end testbench ; 

architecture behavioral of testbench is
	constant CLOCK_PERIOD 	: time := 10 ns;

	constant TB_RING_DEPTH 		: integer := 4;
	constant TB_PIXEL_SIZE		: integer := 8;
	constant TB_DATA_SIZE		: integer := 32;
	constant TB_IMAGE_H_SIZE	: integer := 64;
	constant TB_IMAGE_V_SIZE	: integer := 32;
	constant TB_RAM_ADDR_WIDTH  : integer := 8;
	
	signal tb_rst : std_logic := '0';
	signal tb_clk : std_logic := '0';

	signal tb_aso_in0_data : std_logic_vector(TB_DATA_SIZE-1 downto 0);	
	signal tb_aso_in0_ready : std_logic;
	signal tb_aso_in0_valid : std_logic;

	signal tb_aso_out0_data : std_logic_vector((9*8)-1 downto 0);
	signal tb_aso_out0_ready : std_logic;
	signal tb_aso_out0_valid : std_logic;

	signal tb_aso_out1_data : std_logic_vector(7 downto 0);
	signal tb_aso_out1_ready : std_logic;
	signal tb_aso_out1_valid : std_logic;
	
	signal tb_aso_out2_data : std_logic_vector(31 downto 0);
	signal tb_aso_out2_ready : std_logic;
	signal tb_aso_out2_valid : std_logic;
	signal tb_aso_out2_sop : std_logic;
	signal tb_aso_out2_eop : std_logic;

    type char_file_t is file of character;
    file file_VECTORS : char_file_t;
    file file_RESULTS : char_file_t;
	
	component rx_ringBuffer is 
		generic (
			RING_DEPTH 	: integer;		
			IMAGE_H_SIZE 	: integer;	
			IMAGE_v_SIZE	: integer;
			PIXEL_SIZE 	: integer;		-- bit
			DATA_SIZE	: integer; 	-- bit
		    RAM_DATA_WIDTH  : integer;
		    RAM_ADDR_WIDTH  : integer
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
			aso_out0_valid  : out 	std_logic                         			--  	.valid
		);
	end component rx_ringBuffer;

	component sobel_core is
		generic (
			DATA_WIDTH 	: integer := 8
		);
		port (
			clk: in std_logic;
			rst: in std_logic;

			aso_in0_data : in std_logic_vector((9*8)-1 downto 0);
			aso_in0_ready : out std_logic;
			aso_in0_valid : in std_logic;

			aso_out0_data 	: out 	std_logic_vector(8-1 downto 0);
			aso_out0_valid	: out 	std_logic;
			aso_out0_ready	: in 	std_logic
  		) ;
  	end component sobel_core ; 

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

	DUT0: rx_ringBuffer
		generic map (
			RING_DEPTH 	=> TB_RING_DEPTH,
			PIXEL_SIZE	=> TB_PIXEL_SIZE,
			DATA_SIZE	=> TB_DATA_SIZE,
			IMAGE_H_SIZE => TB_IMAGE_H_SIZE,
			IMAGE_v_SIZE => TB_IMAGE_V_SIZE,
			RAM_DATA_WIDTH => TB_DATA_SIZE,
			RAM_ADDR_WIDTH => TB_RAM_ADDR_WIDTH
		)
		port map (
			clk	=> tb_clk,
			rst	=> tb_rst,

			aso_in0_data 	=> tb_aso_in0_data,
			aso_in0_ready 	=> tb_aso_in0_ready,
			aso_in0_valid 	=> tb_aso_in0_valid,

			aso_out0_data 	=> tb_aso_out0_data,
			aso_out0_ready 	=> tb_aso_out0_ready,
			aso_out0_valid 	=> tb_aso_out0_valid
		);

	DUT1: sobel_core
		generic map (
			DATA_WIDTH 	=> TB_PIXEL_SIZE
		)	
		port map (
			clk	=> tb_clk,
			rst	=> tb_rst,

			aso_in0_data 	=> tb_aso_out0_data,
			aso_in0_ready 	=> tb_aso_out0_ready,
			aso_in0_valid 	=> tb_aso_out0_valid,

			aso_out0_data 	=> tb_aso_out1_data,
			aso_out0_ready 	=> tb_aso_out1_ready,
			aso_out0_valid 	=> tb_aso_out1_valid
		);

	DUT2: tx_ringBuffer
		generic map (
			PIXEL_SIZE 	=> TB_PIXEL_SIZE,
			DATA_SIZE	=> TB_DATA_SIZE,
			PACKET_SIZE	=> (TB_IMAGE_H_SIZE-2)*2
		)
		port map (
			clk	=> tb_clk,
			rst	=> tb_rst,

			aso_in0_data 	=> tb_aso_out1_data,
			aso_in0_ready 	=> tb_aso_out1_ready,
			aso_in0_valid 	=> tb_aso_out1_valid,

			aso_out0_data 	=> tb_aso_out2_data,
			aso_out0_ready 	=> tb_aso_out2_ready,
			aso_out0_valid 	=> tb_aso_out2_valid,
			aso_out0_sop	=> tb_aso_out2_sop,
			aso_out0_eop	=> tb_aso_out2_eop
		);
		
	tb_clk <= not tb_clk after CLOCK_PERIOD/2;
	
stim_proc:  process
    variable v_ILINE     : line;
    variable v_ADD_TERM2 : std_logic_vector(31 downto 0);
    variable char: character;
    variable char0: character;
    variable char1: character;
    variable char2: character;
    variable char3: character;
    variable counter1: integer;
  begin
 
    tb_rst <= '1';
    wait for CLOCK_PERIOD;
    wait for CLOCK_PERIOD;
    tb_rst <= '0';
 
    wait for CLOCK_PERIOD;
    file_open(file_VECTORS, "input.txt",  read_mode);
    
    counter1 := 0;
    
    wait until tb_clk'event and tb_rst = '0';
 
    wait until rising_edge(tb_clk);
    
    while not endfile(file_VECTORS) loop
      read(file_VECTORS, char0);
      read(file_VECTORS, char1);
      read(file_VECTORS, char2);
      read(file_VECTORS, char3);
      
      v_ADD_TERM2(31 downto 24) := CONV_STD_LOGIC_VECTOR(character'POS(char3), 8);
      v_ADD_TERM2(23 downto 16) := CONV_STD_LOGIC_VECTOR(character'POS(char2), 8);
      v_ADD_TERM2(15 downto 8) 	:= CONV_STD_LOGIC_VECTOR(character'POS(char1), 8);
      v_ADD_TERM2(7 downto 0) 	:= CONV_STD_LOGIC_VECTOR(character'POS(char0), 8);
 
      -- Pass the variable to a signal to allow the ripple-carry to use it
      tb_aso_in0_data <= v_ADD_TERM2;
      tb_aso_in0_valid <= '1';
      wait until rising_edge(tb_clk) and tb_aso_in0_ready = '1';
      counter1 := counter1 + 1;
      
      if (counter1 mod 5) = 0 then
        tb_aso_in0_valid <= '0';
        wait for CLOCK_PERIOD*20;
        wait until rising_edge(tb_clk);
        tb_aso_in0_valid <= '1';
      end if;
    end loop;
 
    file_close(file_VECTORS);
    
    --assert false report "simulation ended" severity failure;
     tb_aso_in0_valid <= '0';
    wait;
  end process stim_proc;
  
  process
    variable v_ILINE     : line;
    variable char0: character;
    variable char1: character;
    variable char2: character;
    variable char3: character;
    variable v_ADD_TERM1 : std_logic_vector(31 downto 0);
    variable counter: integer;
  begin
    file_open(file_RESULTS, "control.txt",  read_mode);
 
    tb_aso_out2_ready <= '0';
    wait for CLOCK_PERIOD;
    wait for CLOCK_PERIOD;
    wait until tb_clk'event and tb_rst = '0';   
    wait for CLOCK_PERIOD;
    wait for CLOCK_PERIOD;
    wait for CLOCK_PERIOD;
    
    tb_aso_out2_ready <= '1';
    
    counter := 0;
    
    while not endfile(file_RESULTS) loop
      read(file_RESULTS, char0);
      read(file_RESULTS, char1);
      read(file_RESULTS, char2);
      read(file_RESULTS, char3);
      
      v_ADD_TERM1(31 downto 24) := CONV_STD_LOGIC_VECTOR(character'POS(char3), 8);
      v_ADD_TERM1(23 downto 16) := CONV_STD_LOGIC_VECTOR(character'POS(char2), 8);
      v_ADD_TERM1(15 downto 8) 	:= CONV_STD_LOGIC_VECTOR(character'POS(char1), 8);
      v_ADD_TERM1(7 downto 0) 	:= CONV_STD_LOGIC_VECTOR(character'POS(char0), 8);
      
      wait until rising_edge(tb_clk) and tb_aso_out2_valid = '1';
      -- Pass the variable to a signal to allow the ripple-carry to use it
      --assert tb_aso_out2_data = v_ADD_TERM1 report "TEST FALLITO (WORKING ZONE). Expected  found"  severity failure;
      counter := counter + 1;
      
      if (counter mod 5) = 0 then
        tb_aso_out2_ready <= '0';
        wait for CLOCK_PERIOD*20;
        wait until rising_edge(tb_clk);
        tb_aso_out2_ready <= '1';
      end if;
    end loop;
    
    tb_aso_out2_ready <= '0';
 
    file_close(file_RESULTS);
     
     assert false report "simulation ended" severity failure;
     
    wait;
  end process;
end Behavioral;