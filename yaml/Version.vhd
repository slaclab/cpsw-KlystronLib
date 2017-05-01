------------------------------------------------------------------------------
-- This file is part of 'LCLS2 KLYSTRON Development'.
-- It is subject to the license terms in the LICENSE.txt file found in the 
-- top-level directory of this distribution and at: 
--    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. 
-- No part of 'LCLS2 KLYSTRON Development', including this file, 
-- may be copied, modified, propagated, or distributed except according to 
-- the terms contained in the LICENSE.txt file.
------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

package Version is

   constant FPGA_VERSION_C : std_logic_vector(31 downto 0) := x"00000006";  -- MAKE_VERSION

   constant BUILD_STAMP_C : string := "AmcCarrierMrEth2x: Vivado v2016.1 (x86_64) Built Wed Jul 27 09:00:49 PDT 2016 by mdewart";

end Version;

-------------------------------------------------------------------------------
-- Revision History:
--
-- 04/20/2016 (0x00000001): Initial Build
-- 07/18/2016 (0x00000002): DaqMuxV2 and hwBay0 and hwBay1 triggered from Timing
-- 07/21/2016 (0x00000003): Timing bug fixes
-- 07/21/2016 (0x00000004): Add timing debug to daqmux 
-- 07/21/2016 (0x00000005): Fix MIG, change RTM digital out port 
-------------------------------------------------------------------------------

