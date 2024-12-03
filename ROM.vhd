-- VHDL netlist generated by SCUBA Diamond (64-bit) 3.12.1.454
-- Module  Version: 5.4
--C:\lscc\diamond\3.12\ispfpga\bin\nt64\scuba.exe -w -n ROM -lang vhdl -synth synplify -bus_exp 7 -bb -arch xo2c00 -type bram -wp 00 -rp 1100 -addr_width 12 -data_width 8 -num_rows 4096 -cascade 11 -memfile c:/kevan/a3051/p3051/rom.mem -memformat hex 

-- Tue Dec 03 16:19:47 2024

library IEEE;
use IEEE.std_logic_1164.all;
-- synopsys translate_off
library MACHXO2;
use MACHXO2.components.all;
-- synopsys translate_on

entity ROM is
    port (
        Address: in  std_logic_vector(11 downto 0); 
        OutClock: in  std_logic; 
        OutClockEn: in  std_logic; 
        Reset: in  std_logic; 
        Q: out  std_logic_vector(7 downto 0));
end ROM;

architecture Structure of ROM is

    -- internal signal declarations
    signal scuba_vhi: std_logic;
    signal scuba_vlo: std_logic;

    -- local component declarations
    component VHI
        port (Z: out  std_logic);
    end component;
    component VLO
        port (Z: out  std_logic);
    end component;
    component DP8KC
        generic (INIT_DATA : in String; INITVAL_1F : in String; 
                INITVAL_1E : in String; INITVAL_1D : in String; 
                INITVAL_1C : in String; INITVAL_1B : in String; 
                INITVAL_1A : in String; INITVAL_19 : in String; 
                INITVAL_18 : in String; INITVAL_17 : in String; 
                INITVAL_16 : in String; INITVAL_15 : in String; 
                INITVAL_14 : in String; INITVAL_13 : in String; 
                INITVAL_12 : in String; INITVAL_11 : in String; 
                INITVAL_10 : in String; INITVAL_0F : in String; 
                INITVAL_0E : in String; INITVAL_0D : in String; 
                INITVAL_0C : in String; INITVAL_0B : in String; 
                INITVAL_0A : in String; INITVAL_09 : in String; 
                INITVAL_08 : in String; INITVAL_07 : in String; 
                INITVAL_06 : in String; INITVAL_05 : in String; 
                INITVAL_04 : in String; INITVAL_03 : in String; 
                INITVAL_02 : in String; INITVAL_01 : in String; 
                INITVAL_00 : in String; ASYNC_RESET_RELEASE : in String; 
                RESETMODE : in String; GSR : in String; 
                WRITEMODE_B : in String; WRITEMODE_A : in String; 
                CSDECODE_B : in String; CSDECODE_A : in String; 
                REGMODE_B : in String; REGMODE_A : in String; 
                DATA_WIDTH_B : in Integer; DATA_WIDTH_A : in Integer);
        port (DIA8: in  std_logic; DIA7: in  std_logic; 
            DIA6: in  std_logic; DIA5: in  std_logic; 
            DIA4: in  std_logic; DIA3: in  std_logic; 
            DIA2: in  std_logic; DIA1: in  std_logic; 
            DIA0: in  std_logic; ADA12: in  std_logic; 
            ADA11: in  std_logic; ADA10: in  std_logic; 
            ADA9: in  std_logic; ADA8: in  std_logic; 
            ADA7: in  std_logic; ADA6: in  std_logic; 
            ADA5: in  std_logic; ADA4: in  std_logic; 
            ADA3: in  std_logic; ADA2: in  std_logic; 
            ADA1: in  std_logic; ADA0: in  std_logic; CEA: in  std_logic; 
            OCEA: in  std_logic; CLKA: in  std_logic; WEA: in  std_logic; 
            CSA2: in  std_logic; CSA1: in  std_logic; 
            CSA0: in  std_logic; RSTA: in  std_logic; 
            DIB8: in  std_logic; DIB7: in  std_logic; 
            DIB6: in  std_logic; DIB5: in  std_logic; 
            DIB4: in  std_logic; DIB3: in  std_logic; 
            DIB2: in  std_logic; DIB1: in  std_logic; 
            DIB0: in  std_logic; ADB12: in  std_logic; 
            ADB11: in  std_logic; ADB10: in  std_logic; 
            ADB9: in  std_logic; ADB8: in  std_logic; 
            ADB7: in  std_logic; ADB6: in  std_logic; 
            ADB5: in  std_logic; ADB4: in  std_logic; 
            ADB3: in  std_logic; ADB2: in  std_logic; 
            ADB1: in  std_logic; ADB0: in  std_logic; CEB: in  std_logic; 
            OCEB: in  std_logic; CLKB: in  std_logic; WEB: in  std_logic; 
            CSB2: in  std_logic; CSB1: in  std_logic; 
            CSB0: in  std_logic; RSTB: in  std_logic; 
            DOA8: out  std_logic; DOA7: out  std_logic; 
            DOA6: out  std_logic; DOA5: out  std_logic; 
            DOA4: out  std_logic; DOA3: out  std_logic; 
            DOA2: out  std_logic; DOA1: out  std_logic; 
            DOA0: out  std_logic; DOB8: out  std_logic; 
            DOB7: out  std_logic; DOB6: out  std_logic; 
            DOB5: out  std_logic; DOB4: out  std_logic; 
            DOB3: out  std_logic; DOB2: out  std_logic; 
            DOB1: out  std_logic; DOB0: out  std_logic);
    end component;
    attribute MEM_LPC_FILE : string; 
    attribute MEM_INIT_FILE : string; 
    attribute MEM_LPC_FILE of ROM_0_0_0_3 : label is "ROM.lpc";
    attribute MEM_INIT_FILE of ROM_0_0_0_3 : label is "rom.mem";
    attribute MEM_LPC_FILE of ROM_0_0_1_2 : label is "ROM.lpc";
    attribute MEM_INIT_FILE of ROM_0_0_1_2 : label is "rom.mem";
    attribute MEM_LPC_FILE of ROM_0_0_2_1 : label is "ROM.lpc";
    attribute MEM_INIT_FILE of ROM_0_0_2_1 : label is "rom.mem";
    attribute MEM_LPC_FILE of ROM_0_0_3_0 : label is "ROM.lpc";
    attribute MEM_INIT_FILE of ROM_0_0_3_0 : label is "rom.mem";
    attribute NGD_DRC_MASK : integer;
    attribute NGD_DRC_MASK of Structure : architecture is 1;

begin
    -- component instantiation statements
    ROM_0_0_0_3: DP8KC
        generic map (INIT_DATA=> "STATIC", ASYNC_RESET_RELEASE=> "SYNC", 
        INITVAL_1F=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1E=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1D=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1C=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1B=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1A=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_19=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_18=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_17=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_16=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_15=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_14=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_13=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_12=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_11=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_10=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0F=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0E=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0D=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0C=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0B=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0A=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_09=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_08=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_07=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_06=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_05=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_04=> "0x00000000000000000000000000000000000000000049F012520D88A0D2420802700844020A502859", 
        INITVAL_03=> "0x1E22307CC316218120DA0124208A4C04C9009E320D2020C8820E6820D28208230032470A0E512449", 
        INITVAL_02=> "0x1CC920F2A413C6904E9A092E6124791489E0D227134491CC920B22404A920845904825124420B224", 
        INITVAL_01=> "0x04A9208459048251245012C791489E0D225124420B22404A920845904825124420B22404A9208459", 
        INITVAL_00=> "0x04E091CC920B22404A920845904825124420B22404A9208459048251244012C7904AC81D22501055", 
        CSDECODE_B=> "0b111", CSDECODE_A=> "0b000", WRITEMODE_B=> "NORMAL", 
        WRITEMODE_A=> "NORMAL", GSR=> "ENABLED", RESETMODE=> "ASYNC", 
        REGMODE_B=> "NOREG", REGMODE_A=> "NOREG", DATA_WIDTH_B=>  2, 
        DATA_WIDTH_A=>  2)
        port map (DIA8=>scuba_vlo, DIA7=>scuba_vlo, DIA6=>scuba_vlo, 
            DIA5=>scuba_vlo, DIA4=>scuba_vlo, DIA3=>scuba_vlo, 
            DIA2=>scuba_vlo, DIA1=>scuba_vlo, DIA0=>scuba_vlo, 
            ADA12=>Address(11), ADA11=>Address(10), ADA10=>Address(9), 
            ADA9=>Address(8), ADA8=>Address(7), ADA7=>Address(6), 
            ADA6=>Address(5), ADA5=>Address(4), ADA4=>Address(3), 
            ADA3=>Address(2), ADA2=>Address(1), ADA1=>Address(0), 
            ADA0=>scuba_vlo, CEA=>OutClockEn, OCEA=>OutClockEn, 
            CLKA=>OutClock, WEA=>scuba_vlo, CSA2=>scuba_vlo, 
            CSA1=>scuba_vlo, CSA0=>scuba_vlo, RSTA=>Reset, 
            DIB8=>scuba_vlo, DIB7=>scuba_vlo, DIB6=>scuba_vlo, 
            DIB5=>scuba_vlo, DIB4=>scuba_vlo, DIB3=>scuba_vlo, 
            DIB2=>scuba_vlo, DIB1=>scuba_vlo, DIB0=>scuba_vlo, 
            ADB12=>scuba_vlo, ADB11=>scuba_vlo, ADB10=>scuba_vlo, 
            ADB9=>scuba_vlo, ADB8=>scuba_vlo, ADB7=>scuba_vlo, 
            ADB6=>scuba_vlo, ADB5=>scuba_vlo, ADB4=>scuba_vlo, 
            ADB3=>scuba_vlo, ADB2=>scuba_vlo, ADB1=>scuba_vlo, 
            ADB0=>scuba_vlo, CEB=>scuba_vhi, OCEB=>scuba_vhi, 
            CLKB=>scuba_vlo, WEB=>scuba_vlo, CSB2=>scuba_vlo, 
            CSB1=>scuba_vlo, CSB0=>scuba_vlo, RSTB=>scuba_vlo, 
            DOA8=>open, DOA7=>open, DOA6=>open, DOA5=>open, DOA4=>open, 
            DOA3=>open, DOA2=>open, DOA1=>Q(1), DOA0=>Q(0), DOB8=>open, 
            DOB7=>open, DOB6=>open, DOB5=>open, DOB4=>open, DOB3=>open, 
            DOB2=>open, DOB1=>open, DOB0=>open);

    ROM_0_0_1_2: DP8KC
        generic map (INIT_DATA=> "STATIC", ASYNC_RESET_RELEASE=> "SYNC", 
        INITVAL_1F=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1E=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1D=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1C=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1B=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1A=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_19=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_18=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_17=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_16=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_15=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_14=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_13=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_12=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_11=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_10=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0F=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0E=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0D=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0C=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0B=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0A=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_09=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_08=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_07=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_06=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_05=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_04=> "0x00000000000000000000000000000000000000000040704AC90845800A09080570040402055024C0", 
        INITVAL_03=> "0x0E02200441120440A09108A050800C02A5000C110AA090E4550E20508A410820008A05002140A265", 
        INITVAL_02=> "0x0AA590AA560AA5512A550CA550B2550AC550AA950AA650AA550AA560AA550CA550AC550AA650AA56", 
        INITVAL_01=> "0x0AA550CA550AC550AA600A24502C550AA550AA650AA560AA550CA550AC550AA650AA560AA550CA55", 
        INITVAL_00=> "0x0AE250AA550AA560AA550CA550AC550AA650AA560AA550CA550AC550AA600A245128250AA1400810", 
        CSDECODE_B=> "0b111", CSDECODE_A=> "0b000", WRITEMODE_B=> "NORMAL", 
        WRITEMODE_A=> "NORMAL", GSR=> "ENABLED", RESETMODE=> "ASYNC", 
        REGMODE_B=> "NOREG", REGMODE_A=> "NOREG", DATA_WIDTH_B=>  2, 
        DATA_WIDTH_A=>  2)
        port map (DIA8=>scuba_vlo, DIA7=>scuba_vlo, DIA6=>scuba_vlo, 
            DIA5=>scuba_vlo, DIA4=>scuba_vlo, DIA3=>scuba_vlo, 
            DIA2=>scuba_vlo, DIA1=>scuba_vlo, DIA0=>scuba_vlo, 
            ADA12=>Address(11), ADA11=>Address(10), ADA10=>Address(9), 
            ADA9=>Address(8), ADA8=>Address(7), ADA7=>Address(6), 
            ADA6=>Address(5), ADA5=>Address(4), ADA4=>Address(3), 
            ADA3=>Address(2), ADA2=>Address(1), ADA1=>Address(0), 
            ADA0=>scuba_vlo, CEA=>OutClockEn, OCEA=>OutClockEn, 
            CLKA=>OutClock, WEA=>scuba_vlo, CSA2=>scuba_vlo, 
            CSA1=>scuba_vlo, CSA0=>scuba_vlo, RSTA=>Reset, 
            DIB8=>scuba_vlo, DIB7=>scuba_vlo, DIB6=>scuba_vlo, 
            DIB5=>scuba_vlo, DIB4=>scuba_vlo, DIB3=>scuba_vlo, 
            DIB2=>scuba_vlo, DIB1=>scuba_vlo, DIB0=>scuba_vlo, 
            ADB12=>scuba_vlo, ADB11=>scuba_vlo, ADB10=>scuba_vlo, 
            ADB9=>scuba_vlo, ADB8=>scuba_vlo, ADB7=>scuba_vlo, 
            ADB6=>scuba_vlo, ADB5=>scuba_vlo, ADB4=>scuba_vlo, 
            ADB3=>scuba_vlo, ADB2=>scuba_vlo, ADB1=>scuba_vlo, 
            ADB0=>scuba_vlo, CEB=>scuba_vhi, OCEB=>scuba_vhi, 
            CLKB=>scuba_vlo, WEB=>scuba_vlo, CSB2=>scuba_vlo, 
            CSB1=>scuba_vlo, CSB0=>scuba_vlo, RSTB=>scuba_vlo, 
            DOA8=>open, DOA7=>open, DOA6=>open, DOA5=>open, DOA4=>open, 
            DOA3=>open, DOA2=>open, DOA1=>Q(3), DOA0=>Q(2), DOB8=>open, 
            DOB7=>open, DOB6=>open, DOB5=>open, DOB4=>open, DOB3=>open, 
            DOB2=>open, DOB1=>open, DOB0=>open);

    ROM_0_0_2_1: DP8KC
        generic map (INIT_DATA=> "STATIC", ASYNC_RESET_RELEASE=> "SYNC", 
        INITVAL_1F=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1E=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1D=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1C=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1B=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1A=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_19=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_18=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_17=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_16=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_15=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_14=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_13=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_12=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_11=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_10=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0F=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0E=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0D=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0C=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0B=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0A=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_09=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_08=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_07=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_06=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_05=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_04=> "0x00000000000000000000000000000000000000000040B0E2FC09E18022140C2470E00402284080C2", 
        INITVAL_03=> "0x16E85008FA07E430220C0E21C0F4E518E11150FC0E21C0FE140ECD80C2D8082000C2040020402041", 
        INITVAL_02=> "0x0081008204020410081008204020410081008204020410081008207008100E04100E040207008207", 
        INITVAL_01=> "0x008100E04100E04020740204100810082040207008207008100E04100E040207008207008100E041", 
        INITVAL_00=> "0x008410081008207008100E04100E040207008207008100E04100E040207402041188F1162C40C030", 
        CSDECODE_B=> "0b111", CSDECODE_A=> "0b000", WRITEMODE_B=> "NORMAL", 
        WRITEMODE_A=> "NORMAL", GSR=> "ENABLED", RESETMODE=> "ASYNC", 
        REGMODE_B=> "NOREG", REGMODE_A=> "NOREG", DATA_WIDTH_B=>  2, 
        DATA_WIDTH_A=>  2)
        port map (DIA8=>scuba_vlo, DIA7=>scuba_vlo, DIA6=>scuba_vlo, 
            DIA5=>scuba_vlo, DIA4=>scuba_vlo, DIA3=>scuba_vlo, 
            DIA2=>scuba_vlo, DIA1=>scuba_vlo, DIA0=>scuba_vlo, 
            ADA12=>Address(11), ADA11=>Address(10), ADA10=>Address(9), 
            ADA9=>Address(8), ADA8=>Address(7), ADA7=>Address(6), 
            ADA6=>Address(5), ADA5=>Address(4), ADA4=>Address(3), 
            ADA3=>Address(2), ADA2=>Address(1), ADA1=>Address(0), 
            ADA0=>scuba_vlo, CEA=>OutClockEn, OCEA=>OutClockEn, 
            CLKA=>OutClock, WEA=>scuba_vlo, CSA2=>scuba_vlo, 
            CSA1=>scuba_vlo, CSA0=>scuba_vlo, RSTA=>Reset, 
            DIB8=>scuba_vlo, DIB7=>scuba_vlo, DIB6=>scuba_vlo, 
            DIB5=>scuba_vlo, DIB4=>scuba_vlo, DIB3=>scuba_vlo, 
            DIB2=>scuba_vlo, DIB1=>scuba_vlo, DIB0=>scuba_vlo, 
            ADB12=>scuba_vlo, ADB11=>scuba_vlo, ADB10=>scuba_vlo, 
            ADB9=>scuba_vlo, ADB8=>scuba_vlo, ADB7=>scuba_vlo, 
            ADB6=>scuba_vlo, ADB5=>scuba_vlo, ADB4=>scuba_vlo, 
            ADB3=>scuba_vlo, ADB2=>scuba_vlo, ADB1=>scuba_vlo, 
            ADB0=>scuba_vlo, CEB=>scuba_vhi, OCEB=>scuba_vhi, 
            CLKB=>scuba_vlo, WEB=>scuba_vlo, CSB2=>scuba_vlo, 
            CSB1=>scuba_vlo, CSB0=>scuba_vlo, RSTB=>scuba_vlo, 
            DOA8=>open, DOA7=>open, DOA6=>open, DOA5=>open, DOA4=>open, 
            DOA3=>open, DOA2=>open, DOA1=>Q(5), DOA0=>Q(4), DOB8=>open, 
            DOB7=>open, DOB6=>open, DOB5=>open, DOB4=>open, DOB3=>open, 
            DOB2=>open, DOB1=>open, DOB0=>open);

    scuba_vhi_inst: VHI
        port map (Z=>scuba_vhi);

    scuba_vlo_inst: VLO
        port map (Z=>scuba_vlo);

    ROM_0_0_3_0: DP8KC
        generic map (INIT_DATA=> "STATIC", ASYNC_RESET_RELEASE=> "SYNC", 
        INITVAL_1F=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1E=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1D=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1C=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1B=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_1A=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_19=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_18=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_17=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_16=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_15=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_14=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_13=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_12=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_11=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_10=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0F=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0E=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0D=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0C=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0B=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_0A=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_09=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_08=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_07=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_06=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_05=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000000", 
        INITVAL_04=> "0x000000000000000000000000000000000000000000007000D00080000000020030600000000060C1", 
        INITVAL_03=> "0x0E00000000000C000050000000020000000000000000006800062000000000010000000800000000", 
        INITVAL_02=> "0x00000000000000000000000000000000000000000000000000000010000002000002000001000001", 
        INITVAL_01=> "0x00000020000020000020000000000000000000100000100000020000020000010000010000002000", 
        INITVAL_00=> "0x00000000000000100000020000020000010000010000002000002000002000000000400000000030", 
        CSDECODE_B=> "0b111", CSDECODE_A=> "0b000", WRITEMODE_B=> "NORMAL", 
        WRITEMODE_A=> "NORMAL", GSR=> "ENABLED", RESETMODE=> "ASYNC", 
        REGMODE_B=> "NOREG", REGMODE_A=> "NOREG", DATA_WIDTH_B=>  2, 
        DATA_WIDTH_A=>  2)
        port map (DIA8=>scuba_vlo, DIA7=>scuba_vlo, DIA6=>scuba_vlo, 
            DIA5=>scuba_vlo, DIA4=>scuba_vlo, DIA3=>scuba_vlo, 
            DIA2=>scuba_vlo, DIA1=>scuba_vlo, DIA0=>scuba_vlo, 
            ADA12=>Address(11), ADA11=>Address(10), ADA10=>Address(9), 
            ADA9=>Address(8), ADA8=>Address(7), ADA7=>Address(6), 
            ADA6=>Address(5), ADA5=>Address(4), ADA4=>Address(3), 
            ADA3=>Address(2), ADA2=>Address(1), ADA1=>Address(0), 
            ADA0=>scuba_vlo, CEA=>OutClockEn, OCEA=>OutClockEn, 
            CLKA=>OutClock, WEA=>scuba_vlo, CSA2=>scuba_vlo, 
            CSA1=>scuba_vlo, CSA0=>scuba_vlo, RSTA=>Reset, 
            DIB8=>scuba_vlo, DIB7=>scuba_vlo, DIB6=>scuba_vlo, 
            DIB5=>scuba_vlo, DIB4=>scuba_vlo, DIB3=>scuba_vlo, 
            DIB2=>scuba_vlo, DIB1=>scuba_vlo, DIB0=>scuba_vlo, 
            ADB12=>scuba_vlo, ADB11=>scuba_vlo, ADB10=>scuba_vlo, 
            ADB9=>scuba_vlo, ADB8=>scuba_vlo, ADB7=>scuba_vlo, 
            ADB6=>scuba_vlo, ADB5=>scuba_vlo, ADB4=>scuba_vlo, 
            ADB3=>scuba_vlo, ADB2=>scuba_vlo, ADB1=>scuba_vlo, 
            ADB0=>scuba_vlo, CEB=>scuba_vhi, OCEB=>scuba_vhi, 
            CLKB=>scuba_vlo, WEB=>scuba_vlo, CSB2=>scuba_vlo, 
            CSB1=>scuba_vlo, CSB0=>scuba_vlo, RSTB=>scuba_vlo, 
            DOA8=>open, DOA7=>open, DOA6=>open, DOA5=>open, DOA4=>open, 
            DOA3=>open, DOA2=>open, DOA1=>Q(7), DOA0=>Q(6), DOB8=>open, 
            DOB7=>open, DOB6=>open, DOB5=>open, DOB4=>open, DOB3=>open, 
            DOB2=>open, DOB1=>open, DOB0=>open);

end Structure;

-- synopsys translate_off
library MACHXO2;
configuration Structure_CON of ROM is
    for Structure
        for all:VHI use entity MACHXO2.VHI(V); end for;
        for all:VLO use entity MACHXO2.VLO(V); end for;
        for all:DP8KC use entity MACHXO2.DP8KC(V); end for;
    end for;
end Structure_CON;

-- synopsys translate_on
