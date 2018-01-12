#include <lcm_drv.h>
extern LCM_DRIVER ili9806e_dsi_vdo_lcm_drv_tcl;
extern LCM_DRIVER ili9806e_dsi_vdo_lcm_drv_yassy;
extern LCM_DRIVER hx8379c_wvga_dsi_vdo_lcm_drv;
extern LCM_DRIVER nt35512_wvga_dsi_vdo_lcm_drv;
LCM_DRIVER* lcm_driver_list[] = 
{ 
#if defined(ILI9806E_DSI_VDO)
&ili9806e_dsi_vdo_lcm_drv_tcl,
&ili9806e_dsi_vdo_lcm_drv_yassy,
#endif
#if defined(HX8379C_WVGA_DSI_VDO)
	&hx8379c_wvga_dsi_vdo_lcm_drv,
#endif
#if defined(NT35512_WVGA_DSI_VDO)
	&nt35512_wvga_dsi_vdo_lcm_drv,
#endif


};

#define LCM_COMPILE_ASSERT(condition) LCM_COMPILE_ASSERT_X(condition, __LINE__)
#define LCM_COMPILE_ASSERT_X(condition, line) LCM_COMPILE_ASSERT_XX(condition, line)
#define LCM_COMPILE_ASSERT_XX(condition, line) char assertion_failed_at_line_##line[(condition)?1:-1]

unsigned int lcm_count = sizeof(lcm_driver_list)/sizeof(LCM_DRIVER*);
//LCM_COMPILE_ASSERT(0 != sizeof(lcm_driver_list)/sizeof(LCM_DRIVER*));
