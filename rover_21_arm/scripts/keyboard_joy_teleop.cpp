#include <pthread.h>  // <pthread.h>'in çalışması için cmakelists.txt düzenlenmelidir!!!!
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <signal.h>
#include <stdio.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>

using namespace std;

/* thread_str struct yapısı, ikinci thread'e geçirilecek adresleri tutmak için tasarlandı.
İkinci thread'e tek seferde tek adres geçirilebildiği için thread'e geçirilmek istenen adresler bir arada
bulunsun diye bu thread_str yapısına ihtiyaç duyulmuştur. Geçirilmek istenen objelerin adreslerini tutan
thread_str objesinin adresini geçirmek yeterli olacaktır.
*/

typedef struct thread_struct {
  bool* new_msg_ptr;  // Klavyede bir tuşa basılıp basılmadığını bildiren boolean değişkeninin adresi
  bool* ready_ptr;  // Boş mesaj basmak için hazır olunup olunmadığını kontrol eden boolean değişkeninin adresi
  ros::Publisher* msg_pub_ptr;  // Joy message basacak olan publisher objesinin adresi

} thread_str;

/* Bu fonksiyon, basılan klavye karakterine göre joy message'ı doldurur. 6 adet axes için 1 ve -1 seçenekleri,
klavyenin farklı tuşlarına atanmıştır (Axes'ler normalde 1 ile -1 arasında herhangi bir double değer alabilir lakin
sadece 1 ve -1 kullanılarak işlem basitleştirilmiştir).
        12 adet buton için ise 1 seçeneği, klavyenin farklı tuşlarına atanmıştır (Butonlar, aktifleştirilmezse 0
değerini alırlar).
*/

void fill_message(sensor_msgs::Joy& joy_message, char keyboard_char) {
  if (keyboard_char == 'q')
    joy_message.axes[0] = 1;
  else if (keyboard_char == 'w')
    joy_message.axes[0] = -1;
  else if (keyboard_char == 'e')
    joy_message.axes[1] = 1;
  else if (keyboard_char == 'r')
    joy_message.axes[1] = -1;
  else if (keyboard_char == 't')
    joy_message.axes[2] = 1;
  else if (keyboard_char == 'y')
    joy_message.axes[2] = -1;
  else if (keyboard_char == 'o')
    joy_message.axes[3] = 1;
  else if (keyboard_char == 'p')
    joy_message.axes[3] = -1;
  else if (keyboard_char == 'a')
    joy_message.axes[4] = 1;
  else if (keyboard_char == 's')
    joy_message.axes[4] = -1;
  else if (keyboard_char == 'd')
    joy_message.axes[5] = 1;
  else if (keyboard_char == 'f')
    joy_message.axes[5] = -1;

  else if (keyboard_char == 'g')
    joy_message.buttons[0] = 1;
  else if (keyboard_char == 'h')
    joy_message.buttons[1] = 1;
  else if (keyboard_char == 'j')
    joy_message.buttons[2] = 1;
  else if (keyboard_char == 'k')
    joy_message.buttons[3] = 1;
  else if (keyboard_char == 'l')
    joy_message.buttons[4] = 1;
  else if (keyboard_char == 'z')
    joy_message.buttons[5] = 1;
  else if (keyboard_char == 'x')
    joy_message.buttons[6] = 1;
  else if (keyboard_char == 'c')
    joy_message.buttons[7] = 1;
  else if (keyboard_char == 'v')
    joy_message.buttons[8] = 1;
  else if (keyboard_char == 'b')
    joy_message.buttons[9] = 1;
  else if (keyboard_char == 'n')
    joy_message.buttons[10] = 1;
  else if (keyboard_char == 'm')
    joy_message.buttons[11] = 1;
  else
    return;
}

/* Bu fonksiyon test amacıyla yazılmıştır. Herhangi bir joy message'ın içeriğini bastırır.
Program düzgün çalışıyorsa, basılan mesajı /joy topic'ini dinleyerek de görebilirsiniz.
*/

void print_message(sensor_msgs::Joy joy_message) {
  cout << "Axes: ";
  cout << joy_message.axes[0] << " ";
  cout << joy_message.axes[1] << " ";
  cout << joy_message.axes[2] << " ";
  cout << joy_message.axes[3] << " ";
  cout << joy_message.axes[4] << " ";
  cout << joy_message.axes[5] << endl;

  cout << "Buttons: ";
  cout << joy_message.buttons[0] << " ";
  cout << joy_message.buttons[1] << " ";
  cout << joy_message.buttons[2] << " ";
  cout << joy_message.buttons[3] << " ";
  cout << joy_message.buttons[4] << " ";
  cout << joy_message.buttons[5] << " ";
  cout << joy_message.buttons[6] << " ";
  cout << joy_message.buttons[7] << " ";
  cout << joy_message.buttons[8] << " ";
  cout << joy_message.buttons[9] << " ";
  cout << joy_message.buttons[10] << " ";
  cout << joy_message.buttons[11] << endl << endl;
}

/* Bu fonksiyon, ek thread'de çalışacak fonksiyondur. Amacı klavyeyi dinlemek, klavyede bir tuşa basılırsa main()'deki
boş mesaj yayınını durdurmak ve dolu mesaj yayınlamaktır. 117,118,119,120 ve 128. satırlardaki kodlar, getchar()
fonksiyonunu karakter alırken enter tuşuna basılmasını beklemeyecek hale getiren OS yöntemlerinden oluşur.
*/

void* read_keyboard(void* ptr) {
  int i;
  char keyboard_char;

  static struct termios oldt, newt;

  sensor_msgs::Joy empty_joy_message;
  sensor_msgs::Joy joy_message;

  for (i = 0; i < 6; i++) empty_joy_message.axes.push_back(0.0);  // Boş joy message objesi, 0'larla doldurulur.
  for (i = 0; i < 12; i++) empty_joy_message.buttons.push_back(0);

  thread_str* my_info_str = (thread_str*)ptr;  // ptr pointer'ının casting (türünü belirtme) işlemi

  while (1) {
    *(my_info_str->ready_ptr) =
        false;  // main()' e boş mesaj basmaması gerektiği gönderilir (Çünkü klavye veri okumaya hazır değildir).

    joy_message = empty_joy_message;  // joy_message, doldurulmadan önce boş mesaj olmalıdır (Çünkü doldurulmadan önce
                                      // boş olduğundan emin olunmalıdır).

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    *(my_info_str->ready_ptr) =
        true;  // Klavyenin okumaya hazır olduğu main()'e bildirilir ve main() boş mesaj basmaya devam eder

    keyboard_char = getchar();  // Klavyede bir tuşa basılana kadar thread bu satırda bekler ve main() fonksiyonu boş
                                // mesaj basmaya devam eder.

    *(my_info_str->new_msg_ptr) = true;  // Klavyede bir tuşa basılınca dolu mesaj geleceği main()'e bildirilir ve
                                         // main() boş mesaj yayınlamayı durdurur.

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    fill_message(joy_message, keyboard_char);  // Basılan tuşa göre mesaj doldurulur

    // print_message(joy_message);

    (*(my_info_str->msg_pub_ptr)).publish(joy_message);  // /joy topic'ine mesaj yayınlanır

    *(my_info_str->new_msg_ptr) =
        false;  // Dolu mesaj yayınlandıktan sonra main()'e boş mesaj yayınlayabileceği bildirilir.
  }

  return NULL;
}

int main(int argc, char* argv[]) {
  long long int i;
  char c;
  bool new_message = false;
  bool empty_ready = true;
  thread_str info_str;

  ros::init(argc, argv, "keyboard_joy_teleop");  // Node ismine keyboard_joy_teleop verilir.
  ros::NodeHandle pub_handle;

  sensor_msgs::Joy empty_joy_message;

  for (i = 0; i < 6; i++) empty_joy_message.axes.push_back(0.0);  // Boş joy message 0'larla doldurulur.
  for (i = 0; i < 12; i++) empty_joy_message.buttons.push_back(0);

  ros::Publisher joy_msg_pub = pub_handle.advertise<sensor_msgs::Joy>(
      "/joy", 1000);  // /joy topic'ine veri basmak için publisher objesi oluşturulur.

  ros::Rate loop_rate(
      10);  // Loop rate'in 100 den çok daha düşük olması, basılan boş mesaj sayısını ve sıklığını azaltmak içindir.

  info_str.new_msg_ptr = &new_message;  // Ek thread'e geçirmek için, 3 değişkenin adresi info_str'da depolanır
  info_str.ready_ptr = &empty_ready;
  info_str.msg_pub_ptr = &joy_msg_pub;

  pthread_t id;
  pthread_create(
      &id, NULL, read_keyboard,
      (void*)&info_str);  // info_str'ın adresi thread'e geçirilir ve read_keyboard(), ek thread'de çalışmaya başlar.

  while (ros::ok()) {  // roscore açık olduğu sürece veya program çalıştığı sürece döngü döner.

    if ((!new_message) && (empty_ready)) {  // Dolu mesaj yayınlanmazsa ve klavye tuş okumaya hazırsa

      // print_message(empty_joy_message);

      joy_msg_pub.publish(empty_joy_message);  // Boş mesaj yayınlanır
    }

    loop_rate.sleep();
  }

  return 0;
}
