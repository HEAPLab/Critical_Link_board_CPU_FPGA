library ieee ;
	use ieee.std_logic_1164.all ;
	use ieee.numeric_std.all ;

entity sobel_core is
  generic (
    DATA_WIDTH   : integer := 8
  );
    port (
    clk: in std_logic;
    rst: in std_logic;

    aso_in0_data : in std_logic_vector((9*8)-1 downto 0);
    aso_in0_ready : out std_logic;
    aso_in0_valid : in std_logic;

    aso_out0_data   : out   std_logic_vector(8-1 downto 0);
    aso_out0_valid  : out   std_logic;
    aso_out0_ready  : in   std_logic
    ) ;
end sobel_core ; 

architecture rtl of sobel_core is
	type A_type is array(0 to 3-1, 0 to 3-1) of unsigned(8-1 downto 0);
	signal A : A_type;
	signal A_p : A_type;
	signal Gx : signed(16-1 downto 0);
	signal Gx_p : signed(16-1 downto 0);
	signal Gy : signed(16-1 downto 0);
	signal Gy_p : signed(16-1 downto 0);
	signal Gx_abs : unsigned(16-1 downto 0);
	signal Gx_abs_p : unsigned(16-1 downto 0);
	signal Gy_abs : unsigned(16-1 downto 0);
	signal Gy_abs_p : unsigned(16-1 downto 0);
	signal sum : unsigned(16-1 downto 0);
	signal sum_p : unsigned(16-1 downto 0);

	signal valid 		: std_logic;
	signal valid_p 		: std_logic;
	signal valid_pp 	: std_logic;
	signal valid_ppp 	: std_logic;
	signal valid_pppp 	: std_logic;

	signal s_ready_passthrough : std_logic;

begin

	A(0,0) <= unsigned(aso_in0_data(71 downto 64));
	A(0,1) <= unsigned(aso_in0_data(63 downto 56));
	A(0,2) <= unsigned(aso_in0_data(55 downto 48));
	A(1,0) <= unsigned(aso_in0_data(47 downto 40));
	A(1,1) <= unsigned(aso_in0_data(39 downto 32));
	A(1,2) <= unsigned(aso_in0_data(31 downto 24));
	A(2,0) <= unsigned(aso_in0_data(23 downto 16));
	A(2,1) <= unsigned(aso_in0_data(15 downto 8));
	A(2,2) <= unsigned(aso_in0_data(7 downto 0));

	-- Stage 1
	Gx <= signed(std_logic_vector( A_p(0,2) + (2 * A_p(1,2)) + A_p(2,2) - A_p(0,0) - (2 * A_p(1,0)) - A_p(2,0) ));

	Gy <= signed(std_logic_vector( A_p(2,0) + (2 * A_p(2,1)) + A_p(2,2) - A_p(0,0) - (2 * A_p(0,1)) - A_p(0,2) ));

	-- Stage 2
	Gx_abs <= unsigned(std_logic_vector(abs(Gx_p)));
	Gy_abs <= unsigned(std_logic_vector(abs(Gy_p)));

	-- Stage 3
	sum <= Gx_abs_p + Gy_abs_p;

	aso_out0_valid <= valid_pppp;

	valid <= aso_in0_valid;

	s_ready_passthrough <= aso_out0_ready;

	aso_in0_ready <= s_ready_passthrough;

	aso_out0_data <=  X"FF" when to_integer(sum_p) >= 255 else std_logic_vector(sum_p(7 downto 0));
	

	pipeline: process(clk, rst)
	begin
		if rst = '1' then
			A_p(0,0) <= to_unsigned(0,8); A_p(0,1) <= to_unsigned(0,8); A_p(0,2) <= to_unsigned(0,8);
			A_p(1,0) <= to_unsigned(0,8); A_p(1,1) <= to_unsigned(0,8); A_p(1,2) <= to_unsigned(0,8);
			A_p(2,0) <= to_unsigned(0,8); A_p(2,1) <= to_unsigned(0,8); A_p(2,2) <= to_unsigned(0,8);
			Gx_p 		<= to_signed(0,16); 
			Gy_p		<= to_signed(0,16);
			Gx_abs_p 	<= to_unsigned(0,16); 
			Gy_abs_p 	<= to_unsigned(0,16);
			sum_p 		<= to_unsigned(0,16);

			valid_p <= '0'; valid_pp <= '0'; valid_ppp <= '0'; valid_pppp <= '0';

		elsif rising_edge(clk) then
            if s_ready_passthrough = '1' then
                A_p <= A;
        
                Gx_p <= Gx; Gx_abs_p <= Gx_abs;
                Gy_p <= Gy; Gy_abs_p <= Gy_abs;
        
                sum_p <= sum;
        
                valid_p <= valid; valid_pp <= valid_p; valid_ppp <= valid_pp; valid_pppp <= valid_ppp;
            end if;
		
		end if;
	end process pipeline;


end architecture ;