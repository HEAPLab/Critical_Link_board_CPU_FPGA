library ieee ;
	use ieee.std_logic_1164.all ;
	use ieee.numeric_std.all ;
	use ieee.math_real.all;
    use ieee.std_logic_unsigned.all;

entity tx_ringBuffer is
	generic (
	    PIXEL_SIZE: integer := 8;
	    DATA_SIZE: integer := 32;
		PACKET_SIZE: integer := 5754
	);

  	port (
		clk		: in std_logic;
		rst	: in std_logic;

		-- Avalon Input Streaming data port
		aso_in0_data   : in  	std_logic_vector(PIXEL_SIZE-1 downto 0);	--  in0.data
        aso_in0_ready  : out 	std_logic; 					   				--     .ready
        aso_in0_valid  : in  	std_logic;                         			--     .valid

		-- Avalon Output Streaming data port
		aso_out0_data   : out  	std_logic_vector(DATA_SIZE-1 downto 0);		--  out0.data
        aso_out0_ready  : in 	std_logic; 					   				--  	.ready
        aso_out0_valid  : out 	std_logic;                         			--  	.valid
		aso_out0_sop	: out	std_logic;									--		.sop
		aso_out0_eop	: out	std_logic									--		.eop

 	);
end tx_ringBuffer ;

architecture rtl of tx_ringBuffer is
    constant PIXEL_PER_WORD: integer := DATA_SIZE/PIXEL_SIZE;
    signal s_index_enable_read: std_logic;
    signal s_index_enable_write: std_logic;
	signal s_index 	: integer range 0 to PACKET_SIZE-1;

	type word_type is array(0 to  PIXEL_PER_WORD - 1) of std_logic_vector(PIXEL_SIZE - 1 downto 0);
	signal word_enable: std_logic;
	signal word: word_type;
	signal stalled: std_logic;
	
	
	type state_in_type is (WAITING_FOR_DATA, TRANSMIT);
    signal state_in	: state_in_type		:= WAITING_FOR_DATA;
begin    
	changeState: process(clk, rst)
	begin
		if rst = '1' then
			state_in <= WAITING_FOR_DATA;
		elsif rising_edge(clk) then
			case state_in is
			     when WAITING_FOR_DATA =>
			         if((s_index mod PIXEL_PER_WORD) = PIXEL_PER_WORD - 1) then
			             state_in <= TRANSMIT;
			         end if;
			     when TRANSMIT =>
			         if aso_out0_ready = '1' then
			             state_in <= WAITING_FOR_DATA;
			         end if;
            end case;
		end if;
	end process changeState;
	
	sopEop: process(clk, rst)
	begin
		if rst = '1' then
			aso_out0_sop <= '0';
			aso_out0_eop <= '0';
		elsif rising_edge(clk) then
			if (s_index mod PACKET_SIZE) = PACKET_SIZE - 1 then
			     aso_out0_eop <= '1';
			else
			     aso_out0_eop <= '0';
			end if;
			if (s_index mod PACKET_SIZE) = 3 then
			     aso_out0_sop <= '1';
			else
			     aso_out0_sop <= '0';
			end if;
		end if;
	end process sopEop;
	
	stateLogic: process(state_in)
	begin
        aso_in0_ready <= '0';
        aso_out0_valid <= '0';
        word_enable <= '0';

		case state_in is
			when WAITING_FOR_DATA =>
			     word_enable <= '1';
			     aso_in0_ready <= '1';
			when TRANSMIT =>
			     aso_out0_valid <= '1';
		end case;
	end process stateLogic;

	wordCounter: process(clk, rst)
	begin
		if rst = '1' then
			s_index <= 0;
		elsif rising_edge(clk) then
            if word_enable = '1' and aso_in0_valid = '1'  then
                s_index <= (s_index + 1) mod PACKET_SIZE;
			end if;
		end if;
	end process wordCounter;

	pixelBuffer: process(clk, rst)
	begin
		if rising_edge(clk) then
            if word_enable = '1' and aso_in0_valid = '1'  then
                word(s_index mod PIXEL_PER_WORD) <=  aso_in0_data;
			end if;
		end if;
	end process pixelBuffer;

    aso_out0_data(7 downto 0) <= word(0);
    aso_out0_data(15 downto 8) <= word(1);
    aso_out0_data(23 downto 16) <= word(2);
    aso_out0_data(31 downto 24) <= word(3);
end architecture rtl;
