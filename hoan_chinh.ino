#include <Arduino.h>
#include <esp32-hal-timer.h>

#define ADC_PIN 32                                //chân nhận tín hiệu sóng vào (dữ liệu 12bit = 4095)
#define DAC_PIN 25                                //chân xuất tín hiệu sóng ra  (dữ liệu 8bit = 255)
#define NEXTION_RX 16                             // ESP32 nhận từ TX của Nextion
#define NEXTION_TX 17                             // ESP32 gửi tới RX của Nextion


//-------------------------------------------------------------------------------------------------------------------------------------------------------
/*thay đổi tigan bật tắt timer ở timeralarm và cách để thay đổi là thay đổi bằng số tick hoạt f_mongmuon.
Nếu muốn thay đổi bằng số tick (T) thì chỉ cần thay đổi thay câu lệnh timeralarm thành: timerAlarm(timer, sotick, true, 0);
Nếu muốn thay đổi bằng tần số (f_mongmuon) thì chỉ cần thay đổi câu lệnh thành: timerAlarm(timer, Landem, true, 0);
*/
 const uint32_t Hz = 1000000;                      // tick = 1us số này nên để yên (vì là phần cứng của esp)
 const uint32_t sotick=1000;                       // đổi thời gian timer bât (cách 1 - thay đổi T) thay đổi thẳng trên timerAlarm
 const uint32_t f_mongmuon= 20000;                 // đổi thời gian timer bật (cách 2 - thay đổi f) qua trung gian tính số Landem (= số tick)
//----------------------------------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------------------------------
/* Nếu sử dụng trực tiếp bằng số tick thì ở công thức tính mẫu ta thay thành: somau1chuky = (1000000/sotick)/f_sin;                             
còn nếu sử dụng f_mongmuon thì công thức tính mẫu thành: somau1chuky = (1000000 / Landem)/f_sin;                  */
 const int f_sin = 50;                                           // tần số sóng vào (sóng sin) 
 const uint32_t Landem = (1000000/f_mongmuon);                   // công thức tính số tick nếu mình sử dụng cách nhập f_mongmuon
 const int somau1chuky = (1000000 / Landem) / f_sin;             // tính ra số mẫu mỗi chu kỳ
 const int tongsomauduoclay = somau1chuky * 10;                   // thay đổi tổng số chu kỳ lấy mẫu 
 const int mau_snapshot = somau1chuky;
 
//-----------------------------------------------------------------------------------------------------------------------------------------------------------


// các biến dùng trong TIMER (ISR) nên khai báo bằng volatile, để mỗi lần ISR cập nhật thì dữ liệu ít bị sai sót------------------------------------------
 volatile uint16_t samples[tongsomauduoclay];               // khai báo mảng, mảng sẽ có số (tongsomauduoclay) phần tử ... vd: tongsomauduoclay=200 thì mảng có 200 phần tử
 volatile uint16_t samples2[mau_snapshot]; 
 volatile int vitri = 0;                                   // này để khai báo vị trí nhập mảng
 volatile int vitri_snapshot =0;                           // dành cho sóng
 volatile bool flag_full = false;                          // khai báo dạng dữ liệu cho flag
 volatile bool flag_wave = false;                          // dành cho sóng
 volatile bool flag_wave1 = false;                          // dành cho sóng
 volatile bool flag_wave2 = false;                         // dành cho sóng
 volatile uint16_t value_ADC = 0;                          // khai báo dạng dữ liệu cho ADC 16bit do ko có uint12_t
 volatile uint8_t  value_DAC = 0;                          // khai báo dạng dữ liệu cho DAC 8bit 
 hw_timer_t* timer = nullptr;                              // khai báo timer
//------------------------------------------------------------------------------------------------------------------------------------------------------------



//Khai báo các biến để tính các chức năng trong loop-------------------------------------------------------------------------------------------------------------------------------------------------------------
 float so_bit_cua_dc_offset            = 1706.6;          // khai báo dc_offset (1.5V) - 1706 do hiệu chỉnh lại
 float tong_gia_tri_cac_phan_tu_DC     = 0;
 float tong_cac_phan_tu_AC_binh_phuong = 0;
 float gia_tri_tb_1_phan_tu_DC         = 0;
 float DC_MEAN                         = 0;
 float gia_tri_1_phan_tu_AC            = 0;
 float VAC_RMS                         = 0;
 float scale = (3.202f/4094.5f)           ;
 float song_AC_RMS                     = 0;
  
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//Khai báo thời gian để gửi tín hiệu lên nextion---------------------------------------------------------------------------------------------------------------------------
 unsigned long capnhat_nextion = 0;                 // lần cập nhật1
 unsigned long capnhat_nextion2 = 0;                // lần cập nhật2
 unsigned long capnhat_nextion3 =0;                 // lần cập nhật3
 const unsigned long tgian_capnhat = 500;           // cập nhật Nextion mỗi 500ms
 const unsigned long tgian_capnhatwave = 1;        // cập nhật Nextion wave mỗi 10ms
 static int dem = 0;
 static unsigned long capnhat_nextion4 = 0;
 static unsigned long capnhat_nextion5 = 0;
 const unsigned long  stepwave_ms = 2;   // 2 ms / điểm -> giãn rất đẹp
 static int vitri_ve = 0;
 
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//function sẽ được gọi ra mỗi lần có interrupt (nhiều chức năng bên trong thêm bớt tùy ý)-------------------------------------------------------------------------------------------------------------------
void IRAM_ATTR onTimer() 
{   
  //function1: dùng để tạo ra sóng Vout từ Vin (chức năng bắt buộc có)---------------------------------------------------------------------------------- 
    value_ADC = analogRead(ADC_PIN);               // đọc tín hiệu analog của V tại chân ADC rồi gán vào value_ADC  
  //  
    flag_wave = true;
    flag_wave2 = true;
  //function2: dùng cho snapshot
   if(!flag_wave1 && vitri_snapshot < mau_snapshot){
    samples2[vitri_snapshot++]=value_ADC;
    if(vitri_snapshot >= mau_snapshot){
      flag_wave1 = true;
      vitri_snapshot =0;
    }
   }
  
  //function3: dùng để tính Vrms (chức năng phụ) ----------------------------------------------------------------------------------------------------  
    samples[vitri++] = value_ADC;           // Lưu giá trị mà ADC vừa đọc được vào các phần tử của bảng

    /* Phần if dùng để xác định khi nào phẩn mảng sample đã được fill đầy,
     và sẽ bật cờ lên thành true, tạm dừng nhập giá trị vào mảng và bắt đầu chức năng tính Vrms ở voidloop*/
     if (vitri >= tongsomauduoclay) {           
        vitri = 0;
        flag_full = true;
     } 
  //--------------------------------------------------------------------------------------------------------------------------------------------------
}
// kết thúc ISR-----------------------------------------------------------------------------------------------------------------------------------------------------------------



//SETUP thông số TIMER và xuất sóng ra DAC----------------------------------------------------------------------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);                                // này để sử dụng serial monitor thôi
    analogSetPinAttenuation(ADC_PIN, ADC_11db);          // set up chân đo ADC:  esp32 có các chế độ đo từ ADC_0db (0-0.8V), ADC_2.5db (0-1.1V), ADC_6db(0-1.5V), ADC_11db(0-3.3V)
    dacWrite(DAC_PIN, 0);                                // tạo setup cho chân DAC và sẽ xuất ra giá trị 0 khi timer đếm lần đâu
  //
    timer = timerBegin(Hz);                              // để Hz=1000000 vì khi để 1000000 thì mỗi tick sẽ là 1 micro giây, nếu thay đổi thì mỗi tick sẽ là số khác, khó kiểm soát
    timerAttachInterrupt(timer, &onTimer);               // tạo hàm ngắt cho timer
    timerAlarm(timer, Landem, true, 0);                  // Timer được gọi khi đếm đủ số tick hoặc Landem, và timer sẽ nhảy lại về 0 sao khi được gọi
  //
   Serial2.begin(921600, SERIAL_8N1, 16, 17);              // tạo chân xuất dữ liệu uart qua màn hình  RX=16, TX=17
}
//-----------------------------------------------------------------------------------------------------------------------------------------------


//tạo hàm chuyển dữ liệu lên màn hình nextion------------------------------------------------------------------------------------------------
void sendToNextion(String cmd) {
    Serial2.print(cmd);
    Serial2.write(0xFF);
    Serial2.write(0xFF);
    Serial2.write(0xFF);
}
//----------------------------------------------------------------------------------------------------------------------------------------------

//tạo hàm nhận thay đổi từ màn hình nextion---------------------------------------------------------------------------------------------------------------------------------------------
void handleNextionCommand() {
    static String cmd = "";

    while (Serial2.available()) {
        char c = Serial2.read();

        // bỏ qua các byte 0xFF nếu có
        if ((uint8_t)c == 0xFF) continue;

        // ghép chuỗi tối đa 4 kí tự
        cmd += c;
        if (cmd.length() > 1) {
            cmd.remove(0, cmd.length() - 1);   // chỉ giữ 1 kí tự cuối
        }

        // nếu 4 kí tự cuối là "SNAP" thì reset snapshot
        if (cmd == "S") { 
          noInterrupts();          
            flag_wave1     = false;
            vitri_snapshot = 0;
            vitri_ve       = 0;   
            interrupts();       

            sendToNextion("cls 6,0");   // xóa waveform snapshot (id 6, ch 0)
        }
    }
}

//=====================================================================================================================================================


void loop() {
  //GỒM CÁC FUNCTION KHI FLAG TRUE
  if (flag_full) {
   flag_full = false;
   //reset biến khi tính lại
   tong_gia_tri_cac_phan_tu_DC     =0;
   tong_cac_phan_tu_AC_binh_phuong =0;

    //FUNCTION1 TÍNH DC -----------------------------------------------------------------------------------------------------------------------------------
     for (int i = 0; i < tongsomauduoclay; i++){
     tong_gia_tri_cac_phan_tu_DC += samples[i];                                                    //lấy tất cả các mẫu trong mảng sample rồi tổng lại     
     }
     float dcvalue=(tong_gia_tri_cac_phan_tu_DC / tongsomauduoclay);
     gia_tri_tb_1_phan_tu_DC = (tong_gia_tri_cac_phan_tu_DC / tongsomauduoclay) -1706.5;           //tính giá trị trung bình 1 phần tử DC và trừ đi offset
     DC_MEAN =  gia_tri_tb_1_phan_tu_DC * scale;                                        //đổi ra VOLT
     
    //FUNCTION2 TÍNH AC RMS--------------------------------------------------------------------------------------------------------------------------------------------     
     for (int i = 0; i < tongsomauduoclay; i++) {
     gia_tri_1_phan_tu_AC = samples[i] - 1706.5 ;                                              //giá trị 1 phàn tử AC và trừ đi offset
     tong_cac_phan_tu_AC_binh_phuong  +=  (gia_tri_1_phan_tu_AC * gia_tri_1_phan_tu_AC);       //lấy tổng bình phương các phần tử AC vừa tính đc       
     }     
     VAC_RMS = sqrtf(tong_cac_phan_tu_AC_binh_phuong  / tongsomauduoclay) * scale*20;  //tính AC_RMS và đổi ra VOLT
     uint8_t song =(uint8_t) VAC_RMS*10;
     String s ="add 5,0," + String(song);
      sendToNextion(s);
     
    //FUNCTION3 gửi lên màn hình nextion  VAC_RMS ---------------------------------------------------------------------------------------------------------------------------------      
     if(millis() - capnhat_nextion > tgian_capnhat) {   //giảm thời gian cập nhật lên nextion do VACrms tính trong tgian rất nhỏ, làm vậy để tránh bị lag màn hình khó nhìn
        capnhat_nextion = millis();                     
        String s = "t0.txt=\"" + String(VAC_RMS, 3) + "\""; 
        sendToNextion(s);
        // không nên xài delay vì delay sẽ ảnh hưởng đến các function khác, dùng if (millis) để chỉ delay trong mỗi function này
     }
    
    //FUNCTION4 gửi lên màn hình nextion DC_MEAN ----------------------------------------------------------------------------------------------------------------------------------
     if(millis() - capnhat_nextion2 > tgian_capnhat){
       capnhat_nextion2 = millis();
       String s = "t2.txt=\"" + String(DC_MEAN,4) +"\"";
       sendToNextion(s);
     }
    
  }
//FUNCTION GỬI LÊN  SÓNG REALTIME-------------------------------------------------------------------------------
  if(flag_wave){
    flag_wave = false;
    //reset biến
    uint16_t value_ADC_moi = value_ADC;
    uint8_t  value_ADC_hienthi = value_ADC_moi >>4;
    
   // gửi lên nextion
       String s = "add 3,0,"+ String(value_ADC_hienthi);       
       sendToNextion(s);
     }


//FUNCTION GỬI SÓNG SNAPSHOT-----------------------------------------------------------------------------------------------
 if(flag_wave1){
   if(millis()-capnhat_nextion4 >stepwave_ms){
    capnhat_nextion4=millis();
    uint8_t v = samples2[vitri_ve] >> 4;  // 0..255
    String s = "add 6,0," + String(v);
    sendToNextion(s);

    vitri_ve++;
    if (vitri_ve >= mau_snapshot) {
    vitri_ve = 0;  // lặp lại chu kỳ
     
    // có thể gửi lệnh clear waveform trước nếu muốn
   }
   
 }    
}

//

handleNextionCommand();
}
