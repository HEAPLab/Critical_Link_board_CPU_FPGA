library ieee ;
	use ieee.std_logic_1164.all;
	use ieee.numeric_std.all;
	use ieee.math_real.all;
    use ieee.std_logic_unsigned.all;

entity rx_ringBuffer is
	generic (
		RING_DEPTH 		: integer := 4;
		PIXEL_SIZE 		: integer := 8;		-- bit
		DATA_SIZE		: integer := 32; 	-- bit
		IMAGE_H_SIZE	: integer := 1920;
		IMAGE_V_SIZE	: integer := 1080;
		RAM_DATA_WIDTH  : integer := 32;
		RAM_ADDR_WIDTH  : integer := 8
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
end rx_ringBuffer ;

architecture rtl of rx_ringBuffer is
    component single_port_ram is
        generic 
        (
            DATA_WIDTH : natural;
            ADDR_WIDTH : natural
        );
    
        port 
        (
            clk		: in std_logic;
            addr	: in natural range 0 to 2**ADDR_WIDTH - 1;
            data	: in std_logic_vector((DATA_WIDTH-1) downto 0);
            we		: in std_logic := '1';
            q		: out std_logic_vector((DATA_WIDTH -1) downto 0)
        );
    END component single_port_ram;
    
    constant RING_DEPTH_VEC: std_logic_vector(positive(ceil(log2(real(RING_DEPTH))))-1 downto 0) := std_logic_vector(to_unsigned(RING_DEPTH-1, positive(ceil(log2(real(RING_DEPTH))))));
    
    function CIRC_CNT(head : integer; tail: integer) return integer is
    begin
      return to_integer(unsigned(std_logic_vector(to_unsigned((head - tail), RING_DEPTH_VEC'length)) and RING_DEPTH_VEC));
    end CIRC_CNT;
    
    function CIRC_SPACE(head : integer; tail: integer) return integer is
    begin
      return CIRC_CNT(tail, head+1);
    end CIRC_SPACE;

	constant TOTAL_IN_PIXEL 	: integer 	:= (IMAGE_H_SIZE * IMAGE_V_SIZE);
	constant TOTAL_OUT_PIXEL 	: integer 	:= (IMAGE_H_SIZE-2) * (IMAGE_V_SIZE-2);
	constant RING_WIDTH 		: integer 	:= IMAGE_H_SIZE;

	type ringBuffer_type is array (0 to RING_DEPTH-1, 0 to 1) of std_logic_vector(RAM_DATA_WIDTH-1 downto 0);
	signal ram_data_out_array  	: ringBuffer_type;
	signal ram_data_out_array_en	: std_logic;

    --signal s_data_in 			: std_logic_vector(DATA_SIZE-1 downto 0);

	type txData_type is array (2 downto 0, 2 downto 0) of std_logic_vector(PIXEL_SIZE-1 downto 0);
	signal s_data_out			: txData_type;
	signal s_data_out_en		: std_logic;

	subtype index_i_type is integer range 0 to RING_DEPTH-1;
	signal head 			: index_i_type;
	signal tail 			: index_i_type;
	signal s_update_head_en : std_logic;
	signal s_update_tail_en : std_logic;
	
	signal i0 			    : index_i_type := 1;
	signal i1 			    : index_i_type := 2;
	signal i2 			    : index_i_type := 3;

	signal j0 			    : std_logic := '0';
	signal j1 			    : std_logic := '0';
	signal j2 			    : std_logic := '0';

	signal s_full 			: std_logic;
	signal s_fill_count 	: index_i_type;

	type state_in_type is (INIT, RECEIVE, UPDATE_HEAD, WAIT_FOR_EMPTY, WAIT_FOR_TRANSMISSION_EOF, RESET_IN);
    signal state_in	: state_in_type		:= RECEIVE;

	type state_out_type is (CHECK_FOR_DATA, PREPARE_TRANSMIT, TRANSMIT, UPDATE_TAIL, RESET_OUT);
	signal state_out : state_out_type 	:= CHECK_FOR_DATA;

	signal s_out_endOfRow 	: std_logic;
	signal s_in_endOfRow	: std_logic;
	signal s_in_eof			: std_logic;
	signal s_out_eof		: std_logic;

	signal s_h_in_pixel_cnt 	: integer range 0 to RING_WIDTH-1 := 0;
	signal s_h_in_pixel_cnt_en 	: std_logic;

	signal s_h_out_pixel_cnt 		: integer range 0 to RING_WIDTH-1 := 0;
	signal s_h_out_pixel_cnt_en 	: std_logic;
	signal s_h_out_pixel_cnt_rst 	: std_logic;

	type ramwe_type is array (0 to RING_DEPTH-1, 0 to 1) of std_logic;
	signal ram_we: ramwe_type;

	type ram_address_type is array (0 to RING_DEPTH-1, 0 to 1) of integer range 0 to 2**RAM_ADDR_WIDTH - 1;
	signal ram_address: ram_address_type;

    constant RAM_MAX_ADDR: integer := 2**RAM_ADDR_WIDTH-1;
	signal write_address: integer range 0 to RAM_MAX_ADDR := 0;
	signal ram_read_address_module_0: integer range 0 to RAM_MAX_ADDR;
	signal ram_read_address_module_1: integer range 0 to RAM_MAX_ADDR;

	signal write_data: std_logic_vector(31 DOWNTO 0);

	signal ram_data_out_column_0_module_0: std_logic_vector (RAM_DATA_WIDTH-1 DOWNTO 0);
	signal ram_data_out_column_0_module_1: std_logic_vector (RAM_DATA_WIDTH-1 DOWNTO 0);

	signal ram_data_out_column_1_module_0: std_logic_vector (RAM_DATA_WIDTH-1 DOWNTO 0);
	signal ram_data_out_column_1_module_1: std_logic_vector (RAM_DATA_WIDTH-1 DOWNTO 0);

	signal ram_data_out_column_2_module_0: std_logic_vector (RAM_DATA_WIDTH-1 DOWNTO 0);
	signal ram_data_out_column_2_module_1: std_logic_vector (RAM_DATA_WIDTH-1 DOWNTO 0);

    -- A ram data out is 32 bit. These decide which slice to take, since a pixel is 8 bit ( 0-7, 8-15, ...)
	signal ram_row_index_j0: integer range 0 to 31;
	signal ram_row_index_j1: integer range 0 to 31;
	signal ram_row_index_j2: integer range 0 to 31;

	signal w_column: std_logic;
	
	signal s_data_out_0_0: std_logic_vector(PIXEL_SIZE-1 downto 0);
	signal s_data_out_0_1: std_logic_vector(PIXEL_SIZE-1 downto 0);
	signal s_data_out_0_2: std_logic_vector(PIXEL_SIZE-1 downto 0);
	
	signal s_data_out_1_0: std_logic_vector(PIXEL_SIZE-1 downto 0);
	signal s_data_out_1_1: std_logic_vector(PIXEL_SIZE-1 downto 0);
	signal s_data_out_1_2: std_logic_vector(PIXEL_SIZE-1 downto 0);
	
	signal s_data_out_2_0: std_logic_vector(PIXEL_SIZE-1 downto 0);
	signal s_data_out_2_1: std_logic_vector(PIXEL_SIZE-1 downto 0);
	signal s_data_out_2_2: std_logic_vector(PIXEL_SIZE-1 downto 0);
	
	signal rows_read: integer range 0 to IMAGE_V_SIZE-1;
	signal rows_written: integer range 0 to IMAGE_V_SIZE-1;
	
	signal eoframe_reset: std_logic;
begin
    GEN_REG:
    for I in 0 to RING_DEPTH-1 generate
		REG0 : single_port_ram
		generic map(
            DATA_WIDTH 	=> RAM_DATA_WIDTH,
            ADDR_WIDTH	=> RAM_ADDR_WIDTH
		)
		port map(
            clk,
            ram_address(I, 0),
			write_data,
			ram_we(I, 0),
			ram_data_out_array(I, 0)
		);
		REG1 : single_port_ram
        generic map(
            DATA_WIDTH 	=> RAM_DATA_WIDTH,
            ADDR_WIDTH	=> RAM_ADDR_WIDTH
		)
		port map(
            clk,
            ram_address(I, 1),
			write_data,
			ram_we(I, 1),
			ram_data_out_array(I, 1)
		);

		ram_we(I, 0) <= '1' when I = head and w_column='0' and ram_data_out_array_en = '1' else '0';
		ram_we(I, 1) <= '1' when I = head and w_column='1' and ram_data_out_array_en = '1' else '0';

		ram_address(I, 0) <= write_address when ram_we(I, 0) = '1' else ram_read_address_module_0;
		ram_address(I, 1) <= write_address when ram_we(I, 1) = '1' else ram_read_address_module_1;
    end generate GEN_REG;

	--s_data_in(7 downto 0) 	<= aso_in0_data(31 downto 24);
    --s_data_in(15 downto 8) 	<= aso_in0_data(23 downto 16);
    --s_data_in(23 downto 16) <= aso_in0_data(15 downto 8);
    --s_data_in(31 downto 24) <= aso_in0_data(7 downto 0);

	aso_out0_data(7 downto 0) 	<= s_data_out(0,0);
    aso_out0_data(15 downto 8) 	<= s_data_out(0,1);
    aso_out0_data(23 downto 16) <= s_data_out(0,2);
    aso_out0_data(31 downto 24) <= s_data_out(1,0);
	aso_out0_data(39 downto 32) <= s_data_out(1,1);
    aso_out0_data(47 downto 40) <= s_data_out(1,2);
    aso_out0_data(55 downto 48) <= s_data_out(2,0);
    aso_out0_data(63 downto 56) <= s_data_out(2,1);
    aso_out0_data(71 downto 64) <= s_data_out(2,2);
    
    write_address <= s_h_in_pixel_cnt / 8;

	s_fill_count 	<= CIRC_CNT(head, tail);
	-- Set the flags
	s_full 		    <= '1' when CIRC_SPACE(head, tail) = 0 else '0';

	-- Ho riempito una riga del ring buffer in lettura quindi passo alla procedura per aggiornare head
	s_in_endOfRow 	<= '1' when s_h_in_pixel_cnt 	= 	(RING_WIDTH - (DATA_SIZE / PIXEL_SIZE)) 		else '0';
	-- Ho terminato di leggere un frame quindi alzo il flag di in_eof per passare alla procedura di reset
	s_in_eof 		<= '1' when rows_read >= IMAGE_V_SIZE-1 								else '0';

	-- Ho termianto di processare una riga del ring buffer quindi passo alla procedura per aggiornare tail
	s_out_endOfRow 	<= '1' when s_h_out_pixel_cnt 	>= 	(RING_WIDTH-2)-1 								else '0';
	-- Ho terminato di processare un frame quindi alzo il flag di out_eof per passare alla procedura di reset
	s_out_eof 		<= '1' when rows_written 		>= 	(IMAGE_V_SIZE-2)-1 							else '0';

	i0 <= tail;
	i1 <= (tail + 1) mod RING_DEPTH;
	i2 <= (tail + 2) mod RING_DEPTH;

	ram_read_address_module_0 <= ((s_h_out_pixel_cnt + 4) / 8) mod (RAM_MAX_ADDR+1);
    ram_read_address_module_1 <= (s_h_out_pixel_cnt) / 8;

	j0 <= '1' when ((s_h_out_pixel_cnt) mod 8) >= 4 else '0';
    j1 <= '1' when ((s_h_out_pixel_cnt + 1) mod 8) >= 4 else '0';
    j2 <= '1' when ((s_h_out_pixel_cnt + 2) mod 8 ) >= 4 else '0';

    w_column <= '1' when ((s_h_in_pixel_cnt) mod 8) >= 4 else '0';

	ram_row_index_j0 <= (s_h_out_pixel_cnt mod 4) * 8;
    ram_row_index_j1 <= ((s_h_out_pixel_cnt+1) mod 4) * 8;
    ram_row_index_j2 <= ((s_h_out_pixel_cnt+2) mod 4) * 8;

    ram_data_out_column_0_module_0 <= ram_data_out_array(i0, 0);
    ram_data_out_column_0_module_1 <= ram_data_out_array(i0, 1);

    ram_data_out_column_1_module_0 <= ram_data_out_array(i1, 0);
    ram_data_out_column_1_module_1 <= ram_data_out_array(i1, 1);

    ram_data_out_column_2_module_0 <= ram_data_out_array(i2, 0);
    ram_data_out_column_2_module_1 <= ram_data_out_array(i2, 1);
    
	-- Update the head pointer
	updateHead: process(clk, rst)
	begin
		if rst = '1' then
			head <= 0;
			rows_read <= 0;
		elsif rising_edge(clk) then
            if eoframe_reset = '1' then
                head <= 0;
                rows_read <= 0;
			elsif s_update_head_en = '1' then
				head <= (head + 1) mod RING_DEPTH;
				rows_read <= (rows_read + 1) mod IMAGE_V_SIZE;
			end if;
		end if;
	end process updateHead;
	
	writeDataProc: process(clk, rst)
	begin
	   write_data <= aso_in0_data; -- SALVO: è ok?
	end process writeDataProc;

	-- Update the tail pointer
	updateTail: process(clk, rst)
	begin
		if rst = '1' then
			tail <= 0;
			rows_written <= 0;
		elsif rising_edge(clk) then
            if eoframe_reset = '1' then
                tail <= 0;
                rows_written <= 0;
			elsif s_update_tail_en = '1' and aso_out0_ready = '1' then
				tail <= (tail + 1) mod RING_DEPTH;
				rows_written <= (rows_written + 1) mod (IMAGE_V_SIZE-2);
			end if;
		end if;
	end process updateTail;

	-- Processes executed on any state change
	in_onStateChange: process(state_in)
	begin
        aso_in0_ready 			<= '0';
        ram_data_out_array_en 	<= '0';
		s_update_head_en 		<= '0';
		s_h_in_pixel_cnt_en 	<= '0';

		case state_in is
            when INIT =>
			when RECEIVE =>
				aso_in0_ready 			<= '1';
				ram_data_out_array_en   <= '1';
				s_h_in_pixel_cnt_en 	<= '1';
			when UPDATE_HEAD =>
				s_update_head_en 		<= '1';
			when WAIT_FOR_EMPTY =>
				-- Niente da fare ora, solo aspettare che si liberi il ring buffer
			when WAIT_FOR_TRANSMISSION_EOF =>
				-- Niente da fare, solo aspettare che termini la trasmissione del frame al modulo successivo
				-- prima di resettare il ring buffer e ripartire con un nuovo frame
				-- Non posso ancora resettare il buffer altrimenti mi si resetta head, di conseguenza
				-- fill count diventa inconsistente e potrei procedere a inviare pacchetti già inviati in uscita
			when RESET_IN =>
				--NOT USED
		end case ;
	end process in_onStateChange;

	out_onStateChange: process(state_out)
	begin
		aso_out0_valid 			<= '0';
		s_update_tail_en 		<= '0';
		s_data_out_en			<= '0';
		s_h_out_pixel_cnt_en 	<= '0';		
		s_h_out_pixel_cnt_rst 	<= '0';
        eoframe_reset <= '0';
		case state_out is
			when CHECK_FOR_DATA =>
				-- Niente da fare, solo aspettare che vi siano
				-- abbastanza dati per iniziare la trasmissione
			when PREPARE_TRANSMIT =>
				s_data_out_en			<= '1';
				s_h_out_pixel_cnt_en 	<= '1';
			when TRANSMIT =>
				aso_out0_valid 			<= '1';
				s_data_out_en			<= '1';
				s_h_out_pixel_cnt_en 	<= '1';
			when UPDATE_TAIL =>
				aso_out0_valid 			<= '1';
				s_update_tail_en 		<= '1';
				s_h_out_pixel_cnt_rst 	<= '1';
			when RESET_OUT =>
				 eoframe_reset <= '1';
		end case;
	end process out_onStateChange;

	h_inPixelCounter: process(clk, rst)
	begin
		if rst = '1' then
			s_h_in_pixel_cnt <= 0;
		elsif rising_edge(clk) then
			if eoframe_reset = '1' then
				s_h_in_pixel_cnt <= 0;
			elsif s_h_in_pixel_cnt_en = '1' and aso_in0_valid = '1' then
				s_h_in_pixel_cnt <= (s_h_in_pixel_cnt + (DATA_SIZE/PIXEL_SIZE)) mod RING_WIDTH;
			end if;
		end if;
	end process h_inPixelCounter;

	h_outPixelCounter: process(clk, rst)
	begin
		if rst = '1' then
			s_h_out_pixel_cnt <= 0;
		elsif rising_edge(clk) then
			if s_h_out_pixel_cnt_rst = '1' or eoframe_reset = '1' then
				s_h_out_pixel_cnt <= 0;
			elsif s_h_out_pixel_cnt_en = '1' and aso_out0_ready = '1' then
				s_h_out_pixel_cnt <= (s_h_out_pixel_cnt + 1) mod (RING_WIDTH-2);
			end if;
		end if;
	end process h_outPixelCounter;

    s_data_out_0_0 <= ram_data_out_column_0_module_0(ram_row_index_j0+PIXEL_SIZE-1 downto ram_row_index_j0) when j0 = '0' else ram_data_out_column_0_module_1(ram_row_index_j0+PIXEL_SIZE-1 downto ram_row_index_j0);
    s_data_out_0_1 <= ram_data_out_column_0_module_0(ram_row_index_j1+PIXEL_SIZE-1  downto ram_row_index_j1) when j1 = '0' else ram_data_out_column_0_module_1(ram_row_index_j1+PIXEL_SIZE-1 downto ram_row_index_j1);
    s_data_out_0_2 <= ram_data_out_column_0_module_0(ram_row_index_j2+PIXEL_SIZE-1  downto ram_row_index_j2) when j2 = '0' else ram_data_out_column_0_module_1(ram_row_index_j2+PIXEL_SIZE-1 downto ram_row_index_j2);

    s_data_out_1_0 <= ram_data_out_column_1_module_0(ram_row_index_j0+PIXEL_SIZE-1  downto ram_row_index_j0) when j0 = '0' else ram_data_out_column_1_module_1(ram_row_index_j0+PIXEL_SIZE-1 downto ram_row_index_j0);
    s_data_out_1_1 <= ram_data_out_column_1_module_0(ram_row_index_j1+PIXEL_SIZE-1  downto ram_row_index_j1) when j1 = '0' else ram_data_out_column_1_module_1(ram_row_index_j1+PIXEL_SIZE-1 downto ram_row_index_j1);
    s_data_out_1_2 <= ram_data_out_column_1_module_0(ram_row_index_j2+PIXEL_SIZE-1  downto ram_row_index_j2) when j2 = '0' else ram_data_out_column_1_module_1(ram_row_index_j2+PIXEL_SIZE-1 downto ram_row_index_j2);

    s_data_out_2_0 <= ram_data_out_column_2_module_0(ram_row_index_j0+PIXEL_SIZE-1  downto ram_row_index_j0) when j0 = '0' else ram_data_out_column_2_module_1(ram_row_index_j0+PIXEL_SIZE-1 downto ram_row_index_j0);
    s_data_out_2_1 <= ram_data_out_column_2_module_0(ram_row_index_j1+PIXEL_SIZE-1  downto ram_row_index_j1) when j1 = '0' else ram_data_out_column_2_module_1(ram_row_index_j1+PIXEL_SIZE-1 downto ram_row_index_j1);
    s_data_out_2_2 <= ram_data_out_column_2_module_0(ram_row_index_j2+PIXEL_SIZE-1  downto ram_row_index_j2) when j2 = '0' else ram_data_out_column_2_module_1(ram_row_index_j2+PIXEL_SIZE-1 downto ram_row_index_j2);
    
	out_newData: process(clk, rst)
	begin
		if rst = '1' then
			s_data_out <= ((X"EF", X"EF", X"EF"), (X"EF", X"EF", X"EF"), (X"EF", X"EF", X"EF"));
		elsif rising_edge(clk) then
			if aso_out0_ready = '1' and s_data_out_en = '1' then
				s_data_out(0, 0) <= s_data_out_0_2;
				s_data_out(0, 1) <= s_data_out_0_1;
				s_data_out(0, 2) <= s_data_out_0_0;
				
				s_data_out(1, 0) <= s_data_out_1_2;
				s_data_out(1, 1) <= s_data_out_1_1;
				s_data_out(1, 2) <= s_data_out_1_0;
								
				s_data_out(2, 0) <= s_data_out_2_2;
                s_data_out(2, 1) <= s_data_out_2_1;
                s_data_out(2, 2) <= s_data_out_2_0;
            end if;
		end if;
	end process out_newData;


	in_changeState: process(clk, rst)
	begin
		if rst = '1' then
			state_in <= INIT;
		elsif rising_edge(clk) then
			case state_in is
			    when INIT =>
                    state_in <= RECEIVE;
                when RECEIVE =>
					if s_in_endOfRow = '1' and aso_in0_valid = '1' then
						if s_full = '1' then
							state_in <= WAIT_FOR_EMPTY;
						else
							state_in <= UPDATE_HEAD;
						end if;
					end if;
                when WAIT_FOR_EMPTY =>
					if s_full = '0' then
						state_in <= UPDATE_HEAD;
					end if;
                when UPDATE_HEAD => --aspetta ultimo E incrementa la head
					-- Nel colpo di clock corrente sto aggiorando head (e row contuner)
					-- quindi per il prossimo colpo di clock posso cambiare stato
                    if s_in_eof = '1' then
                        state_in <= WAIT_FOR_TRANSMISSION_EOF;
                    elsif s_full = '1' then
                       state_in <= WAIT_FOR_EMPTY;
                    else
                        state_in <= RECEIVE;
                    end if;
				when WAIT_FOR_TRANSMISSION_EOF =>
					if eoframe_reset = '1' then
						-- Ho finito di trasmettere i dati al modulo successivo
						-- quindi per il prossimo colpo di clock posso cambiare stato
						state_in <= INIT;
					end if;
				when RESET_IN =>
					state_in <= RECEIVE;
            end case;
		end if;
	end process in_changeState;

	out_changeState: process(clk, rst)
	begin
		if rst = '1' then
			state_out <= CHECK_FOR_DATA;
		elsif rising_edge(clk) then
			case state_out is
				when CHECK_FOR_DATA =>
					if (s_fill_count >= 3) then
						-- Ho accumulato abbastanza righe per  iniziare a trasmettere
						-- le matrici 3x3 dal prossimo colpo di clock
						state_out <= PREPARE_TRANSMIT;
					end if;
				when PREPARE_TRANSMIT =>
				    if aso_out0_ready = '1' then
					   state_out <= TRANSMIT;
					end if;
				when TRANSMIT =>
					if s_out_endOfRow = '1' and aso_out0_ready = '1' then
						-- Ho terminato con una riga, posso passare ad incrementare tail
						-- nel prossimo colpo di clock
						state_out <= UPDATE_TAIL;
					end if;
				when UPDATE_TAIL =>
					-- Nel colpo di clock corrente sto aggiornando tail,
					-- quindi posso preparare il cambio di stato per il prossimo colpo di clock
					if aso_out0_ready = '1' then
                        if s_out_eof = '1' then
                            state_out <= RESET_OUT;
                        else
                            state_out <= CHECK_FOR_DATA;
                        end if;
					end if;
				when RESET_OUT =>
					state_out <= CHECK_FOR_DATA;
			end case;
		end if;
	end process out_changeState;
end architecture rtl;
