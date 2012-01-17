--!
--! Copyright (C) 2011 - 2012 Creonic GmbH
--!
--! This file is part of the Creonic Viterbi Decoder, which is distributed
--! under the terms of the GNU General Public License version 2.
--!
--! @file
--! @brief  Viterbi decoder RAM control
--! @author Markus Fehrenz
--! @date   2011/12/13
--!
--! @details Manage RAM behavior. Write and read data.
--! The decisions are sent to the traceback units
--! It is signaled if the data belongs to acquisition or window phase.
--!

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library dec_viterbi;
use dec_viterbi.pkg_param.all;
use dec_viterbi.pkg_param_derived.all;
use dec_viterbi.pkg_types.all;
use dec_viterbi.pkg_components.all;


entity ram_ctrl is
	port(
	clk       : in std_logic;
	rst       : in std_logic;


	--
	-- Slave data signals, delivers the LLR parity values.
	--
	s_axis_input_tvalid : in  std_logic;
	s_axis_input_tdata  : in  std_logic_vector(NUMBER_TRELLIS_STATES - 1 downto 0);
	s_axis_input_tlast  : in  std_logic;
	s_axis_input_tready : out std_logic;


	--
	-- Master data signals for traceback units, delivers the decision vectors.
	--
	m_axis_output_tvalid       : out std_logic_vector(1 downto 0);
	m_axis_output_tdata        : out t_ram_rd_data;
	m_axis_output_tlast        : out std_logic_vector(1 downto 0);
	m_axis_output_tready       : in  std_logic_vector(1 downto 0);

	-- Signals the traceback unit when the decision bits do not belong to an acquisition.
	m_axis_output_window_tuser : out std_logic_vector(1 downto 0);

	-- Signals whether this is the last decision vector of the window.
	m_axis_output_last_tuser   : out std_logic_vector(1 downto 0);


	--
	-- Slave configuration signals, delivering the configuration data.
	--

	s_axis_ctrl_tvalid : in  std_logic;
	s_axis_ctrl_tdata  : in  std_logic_vector(31 downto 0);
	s_axis_ctrl_tready : out std_logic
);
end entity ram_ctrl;


architecture rtl of ram_ctrl is

	------------------------
	-- Type definition
	------------------------

	--
	-- Record contains runtime configuration.
	-- The input configuration is stored in a register.
	-- It is received from a AXI4-Stream interface from the top entity.
	--
	-- We just receive window_length and acquisition_length from the outside. All
	-- other values are computed internally in order to impreove timing of the core:
	-- m denotes minus,
	-- p denotes plus.
	--
	type trec_runtime_param is record
		window_length           : unsigned(BW_MAX_WINDOW_LENGTH - 1 downto 0);
		acquisition_length      : unsigned(BW_MAX_WINDOW_LENGTH - 1 downto 0);
		window_p_acquisition    : unsigned(BW_MAX_WINDOW_LENGTH - 1 downto 0);
		window_p_acquisition_m1 : unsigned(BW_MAX_WINDOW_LENGTH - 1 downto 0);
		window_p_acquisition_m2 : unsigned(BW_MAX_WINDOW_LENGTH - 1 downto 0);
		window_p_acquisition_m3 : unsigned(BW_MAX_WINDOW_LENGTH - 1 downto 0);
		window_m1               : unsigned(BW_MAX_WINDOW_LENGTH - 1 downto 0);
	end record trec_runtime_param;

	-- Types for finite state machines
	type t_ram_ctrl is (CONFIGURE, START, BEGINNING, RUNNING, ENDING);

	-- RAM controling types
	type t_ram_data    is array (3 downto 0) of std_logic_vector(NUMBER_TRELLIS_STATES - 1 downto 0);
	type t_ram_addr    is array (3 downto 0) of unsigned(BW_MAX_WINDOW_LENGTH - 1  downto 0);
	type t_ram_rd_addr is array (1 downto 0) of unsigned(BW_MAX_WINDOW_LENGTH - 1  downto 0);
	type t_ram_ptr     is array (1 downto 0) of unsigned(1 downto 0);
	type t_ram_ptr_int is array (1 downto 0) of integer range 3 downto 0;

	type t_ram_data_cnt is array (1 downto 0) of integer range 2 * MAX_WINDOW_LENGTH downto 0;


	------------------------
	-- Signal declaration
	------------------------

	signal config          : trec_runtime_param;
	signal ram_ctrl_fsm    : t_ram_ctrl;
	signal en_ram, wen_ram : std_logic_vector(3 downto 0);
	signal addr            : t_ram_addr;
	signal d               : std_logic_vector(NUMBER_TRELLIS_STATES - 1 downto 0);
	signal q               : t_ram_rd_data;
	signal q_reg           : t_ram_data;

	-- ram addess, number and data pointer
	signal write_ram_ptr                                : unsigned(1 downto 0);
	signal read_ram_ptr, read_ram_data_ptr              : t_ram_ptr;
	signal read_ram_data_ptr_d1                         : t_ram_ptr;
	signal write_addr_ptr, read_addr_ptr, last_addr_ptr : unsigned(BW_MAX_WINDOW_LENGTH - 1 downto 0);

	-- control flags
	signal switch_ram, switch_state, ram_writing : boolean;

	-- internal signals of outputs
	signal m_axis_output_tvalid_int     : std_logic_vector(1 downto 0);
	signal m_axis_output_last_tuser_int : std_logic_vector(1 downto 0);
	signal s_axis_input_tready_int      : std_logic;

	-- delay signals of inputs
	signal m_axis_output_tvalid_d1, m_axis_output_tvalid_d2  : std_logic_vector(1 downto 0);

	signal next_traceback, not_next_traceback : integer range 1 downto 0;
	signal data_cnt                           : t_ram_data_cnt;

begin

	s_axis_input_tready     <= '1' when (m_axis_output_tready = "11" or m_axis_output_tvalid_int(0) = '0'
	                                     or m_axis_output_tvalid_int(1) = '0')
	                                     and ram_ctrl_fsm /= CONFIGURE and ram_ctrl_fsm /= ENDING else
	                           '0';

	s_axis_input_tready_int <= '1' when (m_axis_output_tready = "11" or m_axis_output_tvalid_int(0) = '0'
	                                     or m_axis_output_tvalid_int(1) = '0')
	                                     and ram_ctrl_fsm /= CONFIGURE and ram_ctrl_fsm /= ENDING else
	                           '0';

	m_axis_output_last_tuser <= m_axis_output_last_tuser_int;
	m_axis_output_tvalid     <= m_axis_output_tvalid_int;
	m_axis_output_tdata      <= q;


	--
	-- Statemachine handles configuration, write/read to/from ram
	-- and forwarding decision vectors to corresponding traceback units
	--
	
	pr_ctrl_ram : process(clk, write_ram_ptr, read_ram_ptr) is
		variable v_write_ram_ptr : integer range 3 downto 0;
		variable v_read_ram_ptr  : t_ram_ptr_int;
	begin
	v_write_ram_ptr   := to_integer(write_ram_ptr);
	v_read_ram_ptr(0) := to_integer(read_ram_ptr(0));
	v_read_ram_ptr(1) := to_integer(read_ram_ptr(1));
	if rising_edge(clk) then
		if rst = '1' then
			ram_ctrl_fsm              <= CONFIGURE;

			config.window_length      <= (others => '0'); 
			config.acquisition_length <= (others => '0');

			en_ram                    <= (others => '1');
			wen_ram                   <= (others => '0');
			addr                      <= (others => (others => '0'));

			write_ram_ptr             <= to_unsigned(0, 2);
			write_addr_ptr            <= (others => '0');
			last_addr_ptr             <= (others => '0');

			read_ram_ptr              <= (to_unsigned(2, 2), to_unsigned(3, 2));
			read_ram_data_ptr         <= (to_unsigned(2, 2), to_unsigned(3, 2));
			read_ram_data_ptr_d1      <= (to_unsigned(2, 2), to_unsigned(3, 2));
			read_addr_ptr             <= (others => '0');
			next_traceback            <= 0;
			not_next_traceback        <= 1;

			switch_ram                <= false;
			switch_state              <= false;
			ram_writing               <= true;

			m_axis_output_last_tuser_int     <= (others => '0');
			m_axis_output_window_tuser       <= (others => '0');
			m_axis_output_tvalid_int  <= (others => '0');
			m_axis_output_tvalid_d1   <= (others => '0');
			m_axis_output_tvalid_d2   <= (others => '0');
			m_axis_output_tlast       <= (others => '0');
			s_axis_ctrl_tready        <= '1';
		else
			addr(v_write_ram_ptr)   <= write_addr_ptr;
			addr(v_read_ram_ptr(0)) <= read_addr_ptr;
			addr(v_read_ram_ptr(1)) <= read_addr_ptr;

			d <= s_axis_input_tdata;

			q(0)   <= q_reg(to_integer(read_ram_data_ptr(0)));
			q(1)   <= q_reg(to_integer(read_ram_data_ptr(1)));

			case ram_ctrl_fsm is

			--
			-- It is necessary to configure the decoder before each block
			-- The decoder is always ready to receive on the ctrl channel, when it is in this state
			--
			when CONFIGURE =>
				if s_axis_ctrl_tvalid = '1' then
					config.window_length      <= unsigned(s_axis_ctrl_tdata(BW_MAX_WINDOW_LENGTH - 1 + 16 downto 16));
					config.acquisition_length <= unsigned(s_axis_ctrl_tdata(BW_MAX_WINDOW_LENGTH - 1      downto  0));
					ram_ctrl_fsm <= START;
					wen_ram(v_write_ram_ptr) <= '1';
					s_axis_ctrl_tready <= '0';
				end if;

			--
			-- After the decoder is configured, the decoder is waiting for a new block
			-- When the AXIS handshake is there the packettransmission begins
			--
			when START =>
				if s_axis_input_tvalid = '1' and s_axis_input_tready_int = '1' then
					ram_ctrl_fsm             <= BEGINNING;
					write_addr_ptr           <= write_addr_ptr + 1;
					read_ram_ptr(next_traceback)                     <= write_ram_ptr - 1;
					read_ram_ptr(not_next_traceback)                 <= write_ram_ptr - 2;
				else
					write_addr_ptr                 <= config.window_length - config.acquisition_length;

					config.window_p_acquisition    <= config.window_length + config.acquisition_length;
					config.window_p_acquisition_m1 <= config.window_length + config.acquisition_length - 1;
					config.window_p_acquisition_m2 <= config.window_length + config.acquisition_length - 2;
					config.window_p_acquisition_m3 <= config.window_length + config.acquisition_length - 3;
					config.window_m1               <= config.window_length - 1;
				end if;

			--
			-- The first arriving data have to be stored on special locations to guarantee
			-- a non malicious behavior of the traceback units.
			--
			when BEGINNING =>
				if s_axis_input_tvalid = '1' and s_axis_input_tready_int = '1' then
					if switch_ram then
						ram_ctrl_fsm <= RUNNING;
						wen_ram(to_integer(unsigned(write_ram_ptr - 1))) <= '0';
						wen_ram(v_write_ram_ptr)                         <= '1';
						write_addr_ptr                                   <= write_addr_ptr + 1;
						read_addr_ptr                                    <= read_addr_ptr  - 1;
						switch_ram                                       <= false;
					else
						if write_addr_ptr = config.window_m1 then
							write_addr_ptr <= to_unsigned(0, BW_MAX_WINDOW_LENGTH);
							read_addr_ptr  <= config.window_m1;
							write_ram_ptr  <= write_ram_ptr + 1;
							switch_ram     <= true;
						else
							write_addr_ptr <= write_addr_ptr + 1;
						end if;
					end if;
				end if;

			--
			-- The running state handles the most decoding
			-- It handels the writing of forward to ram and reading the decision vecotors
			-- the decision vecotrs are forwarded to the traceback units.
			--
			when RUNNING =>
				if (s_axis_input_tvalid = '1' and s_axis_input_tready_int = '1' and ram_writing) or
				   ((m_axis_output_tready(0) = '1' and m_axis_output_tready(1) = '1') and not(ram_writing)) then

					read_ram_data_ptr_d1 <= read_ram_ptr;
					read_ram_data_ptr    <= read_ram_data_ptr_d1;
					m_axis_output_tvalid_int <= m_axis_output_tvalid_d1;
					m_axis_output_tvalid_d1  <= m_axis_output_tvalid_d2;

					if switch_ram then
						if ram_writing then
							wen_ram(to_integer(unsigned(write_ram_ptr - 1))) <= '0';
							wen_ram(v_write_ram_ptr)                         <= '1';
							next_traceback                                   <= not_next_traceback;
							not_next_traceback                               <= next_traceback;
						else
							ram_ctrl_fsm <= ENDING;
							ram_writing <= true;
							wen_ram(to_integer(unsigned(write_ram_ptr - 1))) <= '0';
						end if;
						switch_ram      <= false;
					else
						if write_addr_ptr = config.window_m1 then
							write_ram_ptr                    <= write_ram_ptr + 1;
							read_ram_ptr(next_traceback)     <= write_ram_ptr;
							read_ram_ptr(not_next_traceback) <= write_ram_ptr - 2;
							switch_ram                       <= true;
							data_cnt(next_traceback)         <= 0;
						end if;
					end if;

					-- Store last RAM address and stop writing after last bit of a block arrives
					if s_axis_input_tlast = '1' then
						ram_writing              <= false;
						last_addr_ptr            <= write_addr_ptr;
-- 						wen_ram(v_write_ram_ptr) <= '0';
					end if;

					-- Handle RAM addess point increase and reset
					if write_addr_ptr = config.window_m1 then
						write_addr_ptr <= to_unsigned(0, BW_MAX_WINDOW_LENGTH);
						read_addr_ptr  <= config.window_m1;
					else
						write_addr_ptr <= write_addr_ptr + 1;
						read_addr_ptr  <= read_addr_ptr  - 1;
					end if;

					-- Handle side channel information for traceback units.
					for i in 1 downto 0 loop
						if m_axis_output_tvalid_d1(i) = '1' then
							data_cnt(i) <= data_cnt(i) + 1;
						end if;

						-- Signals the valid signal
				 		if data_cnt(i) = config.window_p_acquisition_m2 then
							m_axis_output_tvalid_d2(i)              <= '0';
						end if;
						if switch_ram and ram_writing then
							m_axis_output_tvalid_d2(next_traceback) <= '1';
						end if;

						-- Signals the end of the current window
						if m_axis_output_last_tuser_int(i) = '1' then
							m_axis_output_last_tuser_int(i) <= '0';
						end if;
						if data_cnt(i) = config.window_p_acquisition_m1 then
							m_axis_output_last_tuser_int(i) <= '1';
						end if;

						-- Signals, whether there is acquisition or window
						if data_cnt(i) = config.window_p_acquisition then
							m_axis_output_window_tuser(i) <= '0';
						end if;
						if data_cnt(i) = config.acquisition_length then
							m_axis_output_window_tuser(i) <= '1';
						end if;
					end loop;
				end if;

			--
			-- Handle last traceback with no acquisition
			-- Maybe the resulting window is longer than others
			--
			when ENDING =>
				if m_axis_output_tready(0) = '1' and m_axis_output_tready(1) = '1' then

					read_ram_data_ptr_d1     <= read_ram_ptr;
					read_ram_data_ptr        <= read_ram_data_ptr_d1;
					m_axis_output_tvalid_int <= m_axis_output_tvalid_d1;

					for i in 1 downto 0 loop
						if m_axis_output_tvalid_int(i) = '1' then
							data_cnt(i) <= data_cnt(i) + 1;
						end if;
					end loop;

					if data_cnt(not_next_traceback) = config.window_p_acquisition_m3 then
						read_addr_ptr <= last_addr_ptr;
					elsif read_addr_ptr = 0 then
						read_ram_ptr(0) <= read_ram_ptr(0) - 1;
						read_ram_ptr(1) <= read_ram_ptr(1) - 1;
						read_addr_ptr   <= config.window_m1;
					else
						read_addr_ptr <= read_addr_ptr  - 1;
					end if;

					if data_cnt(not_next_traceback) =  config.window_p_acquisition_m1 then
						m_axis_output_last_tuser_int(not_next_traceback) <= '1';
						m_axis_output_tvalid_d1(next_traceback)          <= '1';
						m_axis_output_tvalid_d1(not_next_traceback)     <= '0';
					end if;

					if data_cnt(not_next_traceback) = config.window_p_acquisition then
						m_axis_output_window_tuser(not_next_traceback)  <= '0';
						m_axis_output_window_tuser(next_traceback) <= '1';
					end if;

					if data_cnt(not_next_traceback) = config.acquisition_length then
						m_axis_output_window_tuser(not_next_traceback) <= '1';
					end if;

					if m_axis_output_last_tuser_int(not_next_traceback) = '1' then
						m_axis_output_last_tuser_int(not_next_traceback) <= '0';
					end if;

					if data_cnt(next_traceback) = last_addr_ptr + config.acquisition_length - 1 then
						m_axis_output_tvalid_d1(next_traceback)      <= '0';
						m_axis_output_last_tuser_int(next_traceback) <= '1';
						m_axis_output_tlast(next_traceback)          <= '1';
						switch_state                                 <= true;
					end if;

					-- block is finished and the decoder is ready for a new configuration and block
					if switch_state then
						switch_state                 <= false;
						m_axis_output_last_tuser_int <= "00";
						m_axis_output_window_tuser   <= "00";
						m_axis_output_tlast          <= "00";
						ram_ctrl_fsm                 <= CONFIGURE;
						s_axis_ctrl_tready           <= '1';
						data_cnt(0)                  <= 0;
						data_cnt(1)                  <= 0;
						m_axis_output_tvalid_int  <= (others => '0');
						m_axis_output_tvalid_d1   <= (others => '0');
						m_axis_output_tvalid_d2   <= (others => '0');
					end if;
				end if;
			end case;
		end if;
	end if;
	end process pr_ctrl_ram;


	------------------------------
	--- Portmapping components ---
	------------------------------

	gen_generic_sp_ram : for i in 0 to 3 generate
	begin
		inst_generic_sp_ram : generic_sp_ram
	generic map(
		DISTR_RAM => DISTRIBUTED_RAM,
		WORDS     => MAX_WINDOW_LENGTH,
		BITWIDTH  => NUMBER_TRELLIS_STATES
	)
	port map(
		clk => clk,
		rst => rst,
		wen => wen_ram(i),
		en  => en_ram(i),
		a   => std_logic_vector(addr(i)),
		d   => d,
		q   => q_reg(i)
	);
	end generate gen_generic_sp_ram;

end architecture rtl;
