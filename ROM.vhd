-- VHDL netlist generated by SCUBA Diamond (64-bit) 3.12.1.454
-- Module  Version: 5.4
--C:\lscc\diamond\3.12\ispfpga\bin\nt64\scuba.exe -w -n ROM -lang vhdl -synth synplify -bus_exp 7 -bb -arch xo2c00 -type bram -wp 00 -rp 1100 -addr_width 12 -data_width 8 -num_rows 4096 -cascade 11 -memfile c:/kevan/a3051/p3051/rom.mem -memformat hex 

-- Mon Dec 16 14:37:59 2024

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
        INITVAL_08=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000026", 
        INITVAL_07=> "0x0B22412C491409609225124490C8920B22412C4904A920926412459048960924A012641245904896", 
        INITVAL_06=> "0x09225124490C8920B22412C4904A920926413479148A71344A0F2A414E9A09479148A71344A0F2A4", 
        INITVAL_05=> "0x14E9A0B8020B2641245904C9E0D2291CC9213C69052E61249E0D2291CC9213C69052E61222512469", 
        INITVAL_04=> "0x1CC9213C69052E61249E0D2291CC9213C69052E61249E0D20012C590929609225124691CC9213C69", 
        INITVAL_03=> "0x052E61249E0D2291CC9213C69052E61249E0D2220B22613C69052E61249E0D2291CC9213C69052E6", 
        INITVAL_02=> "0x1249E0D2291CC9104A920D2E61249E0D2291CC9213C69052E61249E0D2291CC9213C69000960B24C", 
        INITVAL_01=> "0x0F825092B2062A50120013882128C0120491E2330789C14809110D81209004009112CB0308901AA0", 
        INITVAL_00=> "0x128240A8C20D2041E09A086620C8820E6820D20112226108E013441122240D0C20B8891D22501061", 
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
        INITVAL_08=> "0x00000000000000000000000000000000000000000000000000000000000000000000000000000025", 
        INITVAL_07=> "0x08A150AA550AA550AA550AA550AA550AA550AA550AA550AA550AA550AA5108A050AA550AA550AA55", 
        INITVAL_06=> "0x0AA550AA550AA550AA550AA550AA550AA550A24502C140A26108A16028510C24502C140A26108A16", 
        INITVAL_05=> "0x02851086C108A550AA550A85108A85028580A24510A140B05108A85028580A24510A140A0550AA45", 
        INITVAL_04=> "0x028580A24510A140B05108A85028580A24510A140B05108AF00A2550125108A550AA45028580A245", 
        INITVAL_03=> "0x10A140B05108A85028580A24510A140B05108A050AA540A24510A140B05108A85028580A24510A14", 
        INITVAL_02=> "0x0B05108A85028500AA5508A140B05108A85028580A24510A140B05108A85028580A2451E0510AA0C", 
        INITVAL_01=> "0x0389704A090E01404A010B8090A84A0A0400E0020025C02825010420A00800425000490084501214", 
        INITVAL_00=> "0x0A054000C10AA000C055082090E4550E20508A200A015050410A2600A01402489084500AA1400830", 
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
        INITVAL_07=> "0x082040204100810082040204100810082040204100810082040204100810082E0082040204100810", 
        INITVAL_06=> "0x082040204100810082040204100810082040204100E040207008207008100E04100E040207008207", 
        INITVAL_05=> "0x0081009E50082040204100810082C10081C020411820403810082C10081C020411820403C0402041", 
        INITVAL_04=> "0x0081C020411820403810082C10081C020411820403810082D4020411C01008204020410081C02041", 
        INITVAL_03=> "0x1820403810082C10081C020411820403810082E008204020411820403810082C10081C0204118204", 
        INITVAL_02=> "0x03810082C10081E008100820403810082C10081C020411820403810082C10081C020411A810082E8", 
        INITVAL_01=> "0x05AC71E23C0C0440A28503A42022830220216E8500A1D08871026FF022010F671034A31F831020C7", 
        INITVAL_00=> "0x022C715C5C0E21A11E1C08EFC0FE140ECD80C24603C86012B60304603C840409C09E1A162C40C020", 
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
        INITVAL_06=> "0x00000000000000000000000000000000000000000020000010000010000002000002000001000001", 
        INITVAL_05=> "0x0000000A400000000000000000004000004000000800000800000400000400000080000000000000", 
        INITVAL_04=> "0x00004000000800000800000400000400000080000080000050000000000000000000000000400000", 
        INITVAL_03=> "0x08000008000004000004000000800000800000000000000000080000080000040000040000008000", 
        INITVAL_02=> "0x00800000400000000000000000080000040000040000008000008000004000004000000A0000000C", 
        INITVAL_01=> "0x038030801006000000400188000040000C10E0000000C00000000000000000000000000080000A00", 
        INITVAL_00=> "0x00000020000000000000000000680006200000500000000040000100000208000008000000000020", 
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
