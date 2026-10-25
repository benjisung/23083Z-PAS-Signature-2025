#include "vex.h"
using namespace vex;
vex::color selectedTeamColor = vex::color::black;
int autonoteTask()
{
    // ================================= 設定隊伍顏色 ===================================
    Optical.setLightPower(100, percent);
    while (true)
    {
        // 啟用 Optical Sensor 的光源
        

        // 設定隊伍顏色
    /*    if (Controller1.ButtonLeft.pressing())
        {
            selectedTeamColor = vex::color::red; // 紅隊
            Optical.setLightPower(100, percent); // 開啟燈光
        }
        else if (Controller1.ButtonRight.pressing())
        {
            selectedTeamColor = vex::color::blue; // 藍隊
            Optical.setLightPower(100, percent);  // 開啟燈光
        }
        else if (Controller1.ButtonUp.pressing())
        {
            selectedTeamColor = vex::color::black; // 無隊伍（禁用）
            Optical.setLightPower(0, percent);     // 關閉燈光
        }*/
        if (Optical.isNearObject())
        {

            if (selectedTeamColor != vex::color::black)
            {
                vex::color detectedColor = Optical.color();

                if (selectedTeamColor == vex::color::red)
                {
                    // 紅隊邏輯：非紅色物件排除

                    if (detectedColor != vex::color::blue)
                    {
                        pushCylinder = 0;
                    }
                    else
                    {

                        pushCylinder = 1;
                        wait(0.3, sec);
                        pushCylinder = 0;
                    }
                }
                else if (selectedTeamColor == vex::color::blue)
                {
                    // 藍隊邏輯：非紅色物件正常工作
                    if (detectedColor != vex::color::red)
                    {
                        pushCylinder = 0;
                    }
                    else
                    {
                        
                        pushCylinder = 1;
                        wait(0.3, sec);
                        pushCylinder = 0; // 排除紅色物件
                    }
                }
            }
            else
            {
                // 無隊伍邏輯：所有物件正常吸入
                
            }
        }
    }
    return 0;
}