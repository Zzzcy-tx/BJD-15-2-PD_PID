#include "keyboard.h"
#include "eeprom.h"

float step = 0;
uint16 KeystrokeLabel = 0;
int16 Dispay_Codename = 0;
uint16 display = 1;
uint16 step_back = 0;

const ButtonLabel_enum ButtonLabels[ButtonCnt] =
    {
        ButtonOne,
        ButtonTwo,
        ButtonThree,
        ButtonFour,
        ButtonFive};

//-------------------------------------------------------------------------------------------------------------------
//  @brief      按键扫描
//  @param
//  @param
//  @param
//  @param
//  @return     void
//  @note
//-------------------------------------------------------------------------------------------------------------------

void Keystroke_Scan(void)
{
    static int key_up = 1; // 按键松开标志
    KeystrokeLabel = 0;
    if (key_up && ((KeyONE == 0) || (KeyTWO == 0) || (KeyTHREE == 0) || (KeyFOUR == 0) || (KeyFIVE == 0)))
    {
        // 	if (mode == SINGLE_CLICK)
        // 		key_up = 0;
        // 	if (mode == DOUBLE_CLICK)
        // 		key_up = 1;
        if (KeyONE == 0)
        {
            KeystrokeLabel = 1;
        }
        else if (KeyTWO == 0)
        {
            KeystrokeLabel = 2;
        }
        else if (KeyTHREE == 0)
        {
            KeystrokeLabel = 3;
        }
        else if (KeyFOUR == 0)
        {
            KeystrokeLabel = 4;
        }
        else if (KeyFIVE == 0)
        {
            KeystrokeLabel = 5;
        }
    }
    // if (mode == SINGLE_CLICK && 1 == KeyONE && 1 == KeyTWO && 1 == KeyTHREE && 1 == KeyFOUR)
    // {
    // 	key_up = 1;
    // }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      菜单转换目录
//  @param
//  @params
//  @param
//  @param
//  @return     void
//  @note       启用while来显示目标页面  在每个页面按键按键后都会改变到对应页面，此函数用于更新屏幕显示
//-------------------------------------------------------------------------------------------------------------------
void Keystroke_Menu(void)
{
    switch (Dispay_Codename)
    {
    case 0:
        Keystroke_Menu_HOME();
        break;
    case -1:
        Keystroke_Menu_STEP();
        break;
    case 1:
        Keystroke_Menu_ONE();
        break;
    case 2:
        Keystroke_Menu_TWO();
        break;
    case 3:
        Keystroke_Menu_THREE();
        break;
    case 4:
        Keystroke_Menu_FOUR();
        break;
        // case 5:
        //     Keystroke_Menu_FIVE();
        //    break;
        // case 666:
        //     Keystroke_Menu_RUNNING();
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      显示菜单目录
//  @param
//  @param
//  @param
//  @param
//  @return     void
//  @note       此页面为编号为0
//-------------------------------------------------------------------------------------------------------------------
void Keystroke_Menu_HOME(void)

{
    ips200_Dispaly(BLACK);
    ips200_showstr(0, display % 10, "*");
    ips200_showstr(8, 0, " data________1"); // 1
    ips200_showstr(8, 1, " element_____2"); // 2
    ips200_showstr(8, 2, " adc_get_____3"); // 3
    ips200_showstr(8, 3, " preset______4"); // 4
    ips200_showstr(8, 4, " mdoe");
    ips200_showuint16(40, 4, sp_ready * 10 + 160);
    ips200_showstr(8, 45, " display:    ");
    ips200_showuint16(120, 45, display % 10 + 1);
    ips200_showstr(80, 50, " homepage");
    ips200_showstr(8, 6, " data_reset");
    ips200_showfloat(120, 6, P_S, 4, 3);
    Keystroke_Scan();
    sp_ready += speed3 * 1;
    switch (KeystrokeLabel)
    {
    case KeystrokeONE:
        display -= 1;
        break;
    case KeystrokeTWO:
        display += 1;
        break;
    case KeystrokeTHREE:
        if (display >= 6)
        {
            extern_iap_write_float(124, 3, 2, 0x00);
            extern_iap_write_float(107, 3, 2, 0x10);
            extern_iap_write_float(105, 3, 2, 0x20);
            extern_iap_write_float(-0.19, 1, 4, 0x30);
            extern_iap_write_float(-0.14, 1, 4, 0x40);
            extern_iap_write_float(100.1, 3, 2, 0x50);
            extern_iap_write_float(50.7, 3, 2, 0x60);
            extern_iap_write_float(81, 3, 2, 0x70);
            extern_iap_write_float(13, 3, 2, 0x140);
            extern_iap_write_float(0, 3, 2, 0x150);
            extern_iap_write_float(0, 3, 2, 0x160);
            extern_iap_write_float(0, 3, 2, 0x170);
            extern_iap_write_float(0, 3, 2, 0x180);
            extern_iap_write_float(4200, 5, 1, 0x80);
            extern_iap_write_float(12000, 5, 1, 0x90);
        }
        else
        {
            Dispay_Codename = display + 1;
            display = 0;
            ips200_clear(IPS200_BGCOLOR);
        }

        break;
    case KeystrokeFOUR:
        fanL = 750;
        pwm_duty(PWMB_CH1_P74, fanL);
        delay_ms(900);
        sp_set = sp_ready * 10 + 160;
        // ips200_clear(IPS200_BGCOLOR);
        break;
    }
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      步长页面
//  @param
//  @param
//  @param
//  @param
//  @return     void
//  @note       此页面为编号为0
//-------------------------------------------------------------------------------------------------------------------
void Keystroke_Menu_STEP(void) //-1
                               // Dispay_Codename = -1  display = 0;
{
    ips200_Dispaly(BLACK);
    ips200_showstr(0, display % 10, "*");
    ips200_showstr(8, 0, " Step=0________1"); // 0
    ips200_showstr(8, 1, " Step=0.001____2"); //-1
    ips200_showstr(8, 2, " Step=0.01_____3"); //-2
    ips200_showstr(8, 3, " Step=0.1______4"); //-3
    ips200_showstr(8, 4, " Step=1________5"); //-4
    ips200_showstr(8, 5, " Step=10_______6"); //-5
    ips200_showstr(8, 6, " Step=100______7"); //-6
    ips200_showstr(8, 9, " Step=1000_____8"); //-7
    ips200_showstr(8, 10, " Return pressFOUR");
    ips200_showstr(8, 45, " display");
    ips200_showuint16(120, 45, display % 10);
    ips200_showstr(8, 50, "Step_set");
    ips200_showfloat(120, 50, step, 4, 2);
    Keystroke_Scan();
    switch (KeystrokeLabel)
    {
    case KeystrokeONE:
        display += 1;
        break;
    case KeystrokeTWO:
        display -= 1;
        break;
    case KeystrokeTHREE:
        if (display == 0)
        {
            step = 0;
        }
        else if (display == -1)
        {
            step = 0.001;
        }
        else if (display == -2)
        {
            step = 0.01;
        }
        else if (display == -3)
        {
            step = 0.1;
        }
        else if (display == -4)
        {
            step = 1;
        }
        else if (display == -5)
        {
            step = 10;
        }
        else if (display == -6)
        {
            step = 100;
        }
        else if (display == -7)
        {
            step = 1000;
        }
        else
        {
        }
        break;
    case KeystrokeFOUR:
        Dispay_Codename = step_back;
        display = step_back;
        ips200_clear(IPS200_BGCOLOR);
        break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      菜单一
//  @param
//  @param
//  @param
//  @param
//  @return     void
//  @note       此页面为编号为0
//-------------------------------------------------------------------------------------------------------------------
void Keystroke_Menu_ONE(void) // 1
                              // Dispay_Codename = 1  display = 11
{
    ips200_Dispaly(BLACK);
    ips200_showstr(0, display % 10, "*");
    ips200_showstr(8, 8, " P_ZHI");
    ips200_showfloat(120, 8, P_S, 4, 4);
    ips200_showstr(8, 1, " P_WAN");
    ips200_showfloat(120, 1, P_S_MAX, 4, 4);

    ips200_showstr(8, 2, " D_ZHI");
    ips200_showfloat(120, 2, D_S_z, 4, 4);
    ips200_showstr(8, 3, " D_WAN");
    ips200_showfloat(120, 3, D_S, 4, 4);

    ips200_showstr(8, 4, " G_ZHI");
    ips200_showfloat(120, 5, G_S_zhi, 4, 4);
    ips200_showstr(8, 5, " G_WAN");
    ips200_showfloat(120, 4, G_S_1, 4, 4);

    ips200_showstr(8, 6, " P_LR");
    ips200_showfloat(120, 6, P_LR, 4, 4);
    ips200_showstr(8, 7, " I_LR");
    ips200_showfloat(120, 7, I_LR, 4, 4);

    ips200_showstr(8, 0, " sp_set");
    ips200_showint16(120, 0, sp_set);
    ips200_showstr(8, 9, " k");
    ips200_showfloat(120, 9, k, 4, 4);
    ips200_showstr(8, 10, " gyro");
    ips200_showfloat(120, 10, gyro, 4, 4);

    ips200_showstr(8, 11, " Return");
    ips200_showstr(8, 45, " display");
    ips200_showuint16(120, 45, display % 10);
    Keystroke_Scan();
    switch (display % 10)
    {
    case 0:
        sp_set += speed3 * 6;
        break;
    case 1:
        P_S_MAX += speed3 * 0.6;
        extern_iap_write_float(P_S_MAX, 3, 2, 0x00);
        break;
    case 2:
        D_S_z += speed3 * 0.6;
        extern_iap_write_float(D_S_z, 3, 2, 0x10);
        break;
    case 3:
        D_S += speed3 * 0.6;
        extern_iap_write_float(D_S, 3, 2, 0x20);
        break;
    case 4:
        G_S_1 += speed3 * 0.001;
        extern_iap_write_float(G_S_1, 1, 4, 0x30);
        break;
    case 5:
        G_S_zhi += speed3 * 0.001;
        extern_iap_write_float(G_S_zhi, 1, 4, 0x40);
        break;
    case 6:
        P_LR += speed3 * 0.6;
        extern_iap_write_float(P_LR, 3, 2, 0x50);
        break;
    case 7:
        I_LR += speed3 * 0.6;
        extern_iap_write_float(I_LR, 3, 2, 0x60);
        break;
    case 8:
        P_S += speed3 * 0.6;
        extern_iap_write_float(P_S, 3, 2, 0x70);
        break;
    default:
        break;
    }

    switch (KeystrokeLabel)
    {
    case KeystrokeONE:
        display -= 1;
        break;
    case KeystrokeTWO:
        display += 1;
        break;
    case KeystrokeTHREE:
        // Dispay_Codename = display;
        // display = display*10+1;
        // ips200_clear(IPS200_BGCOLOR);
        break;
    case KeystrokeFOUR:
        Dispay_Codename = 0;
        display = 0;
        ips200_clear(IPS200_BGCOLOR);
        break;
    }
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      菜单二
//  @param
//  @param
//  @param
//  @param
//  @return     void
//  @note       此页面为编号为0
//-------------------------------------------------------------------------------------------------------------------

void Keystroke_Menu_TWO(void) // 2
{
    ips200_Dispaly(BLACK);
    ips200_showstr(0, display % 10, "*");

    ips200_showstr(8, 0, " annulus_flag");
    ips200_showint16(120, 0, annulus_flag);
    ips200_showstr(8, 1, " distance_ring");
    ips200_showint16(120, 1, distance1);
    ips200_showstr(8, 2, " dis1");
    ips200_showint16(120, 2, dis1);
    ips200_showstr(8, 3, " dis2");
    ips200_showint16(120, 3, dis2);
    ips200_showstr(8, 4, " k");
    ips200_showfloat(120, 4, k, 4, 4);
    ips200_showstr(8, 5, " L.setpoint");
    ips200_showfloat(120, 5, pidL.setpoint, 4, 4);
    ips200_showstr(8, 6, " R.setpoint");
    ips200_showfloat(120, 6, pidR.setpoint, 4, 4);
    ips200_showstr(8, 7, " mid_adc_up");
    ips200_showint32(120, 7, adc_data[7], 6);
    ips200_showstr(8, 8, " mid_adc_down");
    ips200_showint32(120, 8, adc_data[2], 6);
    ips200_showstr(8, 10, " flag_k");
    ips200_showuint8(120, 10, annulus_flag_k_index);
    ips200_showstr(8, 11, " angle1");
    ips200_showfloat(120, 11, angle1, 5, 2);
    // ips200_showstr(8, 1, " Return");
    ips200_showstr(8, 45, " display");
    ips200_showuint16(120, 45, display % 10);
    Keystroke_Scan();

    switch (display % 10)
    {
    case 2:
        dis1 -= speed3 * 100;
        extern_iap_write_float((float)dis1, 5, 1, 0x80);
        break;
    case 3:
        dis2 += speed3 * 100;
        extern_iap_write_float((float)dis2, 5, 1, 0x90);
        break;
    default:
        break;
    }

    switch (KeystrokeLabel)
    {
    case KeystrokeONE:
        display -= 1;
        break;
    case KeystrokeTWO:
        display += 1;
        break;
    case KeystrokeTHREE:
        // Dispay_Codename = display;
        // display = display*10+1;
        // ips200_clear(IPS200_BGCOLOR);
        break;
    case KeystrokeFOUR:
        Dispay_Codename = 0;
        display = 0;
        ips200_clear(IPS200_BGCOLOR);
        break;
    }
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      菜单三
//  @param
//  @param
//  @param
//  @param
//  @return     void
//  @note       此页面为编号为0
//-------------------------------------------------------------------------------------------------------------------
void Keystroke_Menu_THREE(void) // 3
{
    ips200_Dispaly(BLACK);

    if (Gui_flag == 0)
    {
        ips200_showstr(0, display % 10, "*");
        ips200_showstr(8, 0, " Adc_0");
        ips200_showint32(120, 0, adc_data[0], 6);
        ips200_showstr(8, 1, " Adc_1");
        ips200_showint32(120, 1, adc_data[1], 6);
        ips200_showstr(8, 2, " Adc_2");
        ips200_showint32(120, 2, adc_data[2], 6);
        ips200_showstr(8, 3, " Adc_3");
        ips200_showint32(120, 3, adc_data[3], 6);
        ips200_showstr(8, 4, " Adc_4");
        ips200_showint32(120, 4, adc_data[4], 6);
        ips200_showstr(8, 6, " Adc_5");
        ips200_showint32(120, 6, adc_data[5], 6);
        ips200_showstr(8, 7, " Adc_6");
        ips200_showint32(120, 7, adc_data[6], 6);
        ips200_showstr(8, 8, " Adc_7");
        ips200_showint32(120, 8, adc_data[7], 6);
        ips200_showstr(8, 9, " Adc_8");
        ips200_showint32(120, 9, adc_data[8], 6);
        ips200_showstr(8, 10, " Adc_9");
        ips200_showint32(120, 10, adc_data[9], 6);
        ips200_showstr(8, 11, " -");
        ips200_showint32(120, 11, adc_data[1] - adc_data[3], 6);
    }
    else
    {
        ips200_showstr(0, display % 10, "*");
        ips200_showstr(8, 0, " Adc_0");
        ips200_showint32(120, 0, adc_sum[0], 6);
        ips200_showstr(8, 1, " Adc_1");
        ips200_showint32(120, 1, adc_sum[1], 6);
        ips200_showstr(8, 2, " Adc_2");
        ips200_showint32(120, 2, adc_sum[2], 6);
        ips200_showstr(8, 3, " Adc_3");
        ips200_showint32(120, 3, adc_sum[3], 6);
        ips200_showstr(8, 4, " Adc_4");
        ips200_showint32(120, 4, adc_sum[4], 6);
        ips200_showstr(8, 6, " Adc_5");
        ips200_showint32(120, 6, adc_sum[5], 6);
        ips200_showstr(8, 7, " Adc_6");
        ips200_showint32(120, 7, adc_sum[6], 6);
        ips200_showstr(8, 8, " Adc_7");
        ips200_showint32(120, 8, adc_sum[7], 6);
        ips200_showstr(8, 9, " Adc_8");
        ips200_showint32(120, 9, adc_sum[8], 6);
        ips200_showstr(8, 10, " Adc_9");
        ips200_showint32(120, 10, adc_sum[9], 6);
        ips200_showstr(8, 11, " t_c");
        ips200_showuint8(120, 11, time_count1);
    }

    ips200_showstr(8, 12, " Gui_flag");
    ips200_showuint16(120, 12, Gui_flag);

    ips200_showstr(8, 45, " display");
    ips200_showuint16(120, 45, display % 10);
    Keystroke_Scan();
    switch (KeystrokeLabel)
    {
    case KeystrokeONE:
        display -= 1;
        break;
    case KeystrokeTWO:
        display += 1;
        break;
    case KeystrokeTHREE:
        Gui_flag++;
        break;
    case KeystrokeFOUR:
        Dispay_Codename = 0;
        display = 0;
        ips200_clear(IPS200_BGCOLOR);
        break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      菜单四
//  @param
//  @param
//  @param
//  @param
//  @return     void
//  @note       此页面为编号为0
//-------------------------------------------------------------------------------------------------------------------
void Keystroke_Menu_FOUR(void) // 4
{
    ips200_Dispaly(BLACK);
    ips200_showstr(0, display % 10, "*");
    ips200_showstr(8, 0, " PO_or_OBSTACLE");
    ips200_showstr(8, 1, " [0]");
    ips200_showint16(80, 1, PO_or_OBSTACLE[0]);
    ips200_showstr(8, 2, " [1]");
    ips200_showint16(80, 2, PO_or_OBSTACLE[1]);
    ips200_showstr(8, 3, " [2]");
    ips200_showint16(80, 3, PO_or_OBSTACLE[2]);
    ips200_showstr(8, 4, " [3]");
    ips200_showint16(80, 4, PO_or_OBSTACLE[3]);
    ips200_showstr(8, 5, " [4]");
    ips200_showint16(80, 5, PO_or_OBSTACLE[4]);
    ips200_showstr(8, 7, " ring_set_k");
    ips200_showstr(8, 8, " [0]");
    ips200_showint16(30, 8, ring_set_k[0]);
    ips200_showstr(8, 9, " [1]");
    ips200_showint16(30, 9, ring_set_k[1]);
    ips200_showstr(8, 10, " [2]");
    ips200_showint16(30, 10, ring_set_k[2]);
    ips200_showstr(8, 11, " [3]");
    ips200_showint16(30, 11, ring_set_k[3]);
    ips200_showstr(8, 12, " [4]");
    ips200_showint16(30, 12, ring_set_k[4]);
    Keystroke_Scan();
    switch (display % 10)
    {
    case 1:
        PO_or_OBSTACLE[0] += speed3 * 1;
        break;
    case 2:
        PO_or_OBSTACLE[1] += speed3 * 1;
        break;
    case 3:
        PO_or_OBSTACLE[2] += speed3 * 1;
        break;
    case 4:
        PO_or_OBSTACLE[3] += speed3 * 1;
        break;
    case 5:
        PO_or_OBSTACLE[4] += speed3 * 1;
        break;
    case 6:
        PO_or_OBSTACLE[5] += speed3 * 1;
        break;
    case 7:
        PO_or_OBSTACLE[6] += speed3 * 1;
        break;
    // case 8:
    //     PO_or_OBSTACLE[7] += speed3 * 1;
    //     break;
    // case 9:
    //     PO_or_OBSTACLE[8] += speed3 * 1;
    //     break;
    // case 10:
    //     PO_or_OBSTACLE[9] += speed3 * 1;
    //     break;
    // case 20:
    //     ring_set_k[0] += speed3 * 0.1;
    //     break;
    // case 21:
    //     ring_set_k[1] += speed3 * 0.1;
    //     break;
    // case 22:
    //     ring_set_k[2] += speed3 * 0.1;
    //     break;
    // case 23:
    //     ring_set_k[3] += speed3 * 0.1;
    //     break;
    // case 24:
    //     ring_set_k[4] += speed3 * 0.1;
    //     break;
    default:
        break;
    }

    switch (KeystrokeLabel)
    {
    case KeystrokeONE:
        display -= 1;
        break;
    case KeystrokeTWO:
        display += 1;
        break;
    case KeystrokeTHREE:
        if (display == 8)
        {
            ring_set_k[0] = fabs(k);
            extern_iap_write_float(ring_set_k[0], 3, 2, 0x140);
        }
        if (display == 9)
        {
            ring_set_k[1] = fabs(k);
            extern_iap_write_float(ring_set_k[1], 3, 2, 0x150);
        }
        if (display == 10)
        {
            ring_set_k[2] = fabs(k);
            extern_iap_write_float(ring_set_k[2], 3, 2, 0x160);
        }
        if (display == 11)
        {
            ring_set_k[3] = fabs(k);
            extern_iap_write_float(ring_set_k[3], 3, 2, 0x170);
        }
        if (display == 12)
        {
            ring_set_k[4] = fabs(k);
            extern_iap_write_float(ring_set_k[4], 3, 2, 0x180);
        }

        // display = display*10+1;
        // ips200_clear(IPS200_BGCOLOR);
        ring_set_k[display % 10] = k;
        break;
    case KeystrokeFOUR:
        Dispay_Codename = 0;
        display = 0;
        ips200_clear(IPS200_BGCOLOR);
        break;
    }
}

/*
//-------------------------------------------------------------------------------------------------------------------
//  @brief      菜单五
//  @param
//  @param
//  @param
//  @param
//  @return     void
//  @note       此页面为编号为0
//-------------------------------------------------------------------------------------------------------------------
void Keystroke_Menu_FIVE(void) // 5
{

}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      发车菜单
//  @param
//  @param
//  @param
//  @param
//  @return     void
//  @note       此页面为编号为0
//-------------------------------------------------------------------------------------------------------------------
void Keystroke_Menu_RUNNING()//
{

}
*/