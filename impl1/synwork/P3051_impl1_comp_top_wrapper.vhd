--
-- Synopsys
-- Vhdl wrapper for top level design, written on Fri Jun 14 14:18:33 2024
--
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity wrapper_for_main is
   port (
      RCK : in std_logic;
      XEN : out std_logic;
      TP1 : out std_logic;
      TP2 : out std_logic;
      SA0 : out std_logic;
      SDA : in std_logic;
      SCL : in std_logic;
      IDY : in std_logic;
      xdac : out std_logic_vector(4 downto 0)
   );
end wrapper_for_main;

architecture behavior of wrapper_for_main is

component main
 port (
   RCK : in std_logic;
   XEN : out std_logic;
   TP1 : out std_logic;
   TP2 : out std_logic;
   SA0 : out std_logic;
   SDA : inout std_logic;
   SCL : inout std_logic;
   IDY : inout std_logic;
   xdac : out std_logic_vector (4 downto 0)
 );
end component;

signal tmp_RCK : std_logic;
signal tmp_XEN : std_logic;
signal tmp_TP1 : std_logic;
signal tmp_TP2 : std_logic;
signal tmp_SA0 : std_logic;
signal tmp_SDA : std_logic;
signal tmp_SCL : std_logic;
signal tmp_IDY : std_logic;
signal tmp_xdac : std_logic_vector (4 downto 0);

begin

tmp_RCK <= RCK;

XEN <= tmp_XEN;

TP1 <= tmp_TP1;

TP2 <= tmp_TP2;

SA0 <= tmp_SA0;

tmp_SDA <= SDA;

tmp_SCL <= SCL;

tmp_IDY <= IDY;

xdac <= tmp_xdac;



u1:   main port map (
		RCK => tmp_RCK,
		XEN => tmp_XEN,
		TP1 => tmp_TP1,
		TP2 => tmp_TP2,
		SA0 => tmp_SA0,
		SDA => tmp_SDA,
		SCL => tmp_SCL,
		IDY => tmp_IDY,
		xdac => tmp_xdac
       );
end behavior;
