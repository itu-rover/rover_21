#include <cmath>
#include <iostream>

using namespace std;

const double PI = 3.14159265;

// Tolerans değeri, FABRIK algoritmasında kullanılmak üzere tanımlanmıştır. Hesaplanan teni hedef noktası
// ile ulaşılmak istenen hedef noktası arasındaki uzaklık, tolerans değerinin altında kalırsa algoritma
// durur (Hata toleransı 0.1 cm'den fazla olduğu sürece algoritma yeni hedef noktasını tekrar hesaplar.).
const double TOLERANCE = 0.1;

// Her bir eklem için açı sınırlandırmaları tanımlanmıştır (Meknaik sınırlandırmaların yazılımsal sınırlandırmalardan
// daha tolere olması sağlıklı olacaktır).

const double J1_ANGLE_BORDER_MAX = 80.0;
const double J1_ANGLE_BORDER_MIN = -80.0;

const double J2_ANGLE_BORDER_MAX = 90.0;
const double J2_ANGLE_BORDER_MIN = -90.0;

const double J3_ANGLE_BORDER_MAX = 70.0;
const double J3_ANGLE_BORDER_MIN = -120.0;

const double J4_ANGLE_BORDER_MAX = 80.0;
const double J4_ANGLE_BORDER_MIN = -80.0;

const double J5_ANGLE_BORDER_MAX = 70.0;
const double J5_ANGLE_BORDER_MIN = -120.0;

// Bu fonksiyon verilen değişkenin mutlak değerini bulur ve döndürür.
// candidate: Mutlak değeri alınacak sayı

double abs_val(double candidate) {
  if (candidate < 0) return candidate * (-1);
  return candidate;
}

// Üç boyutlu uzayda koordinat kavramını tanımlamak için coordinate sınıfı oluşturulmuşur. Bu sınıfa ait objeler, veri
// olarak sadece X,Y ve Z koordinatlarını tutarlar. İhtiyaca göre sınıfa ait farklı metodlar tanımlanmıştır.

class coordinate {
  double x;
  double y;
  double z;

 public:
  coordinate(double = 0.0, double = 0.0, double = 0.0);
  void set_x(double);
  void set_y(double);
  void set_z(double);
  double get_x();
  double get_y();
  double get_z();
  double calculate_dist(coordinate);
  coordinate crd_multiplication(double);
  coordinate operator+(coordinate);
};

/* Bu metod bir değişkenli constructor dır. coordinate sınıfına ait yeni bir obje oluşturulurken 3 adet parametre
verilirse, otomatik olarak çalışır. Verilen parametreleri objenin X,Y ve Z verilerine eşitler.*/
// crd_x: Yeni objenin X koordinatı olacak değer
// crd_y: Yeni objenin Y koordinatı olacak değer
// crd_z: Yeni objenin Z koordinatı olacak değer

coordinate::coordinate(double crd_x, double crd_y, double crd_z) {
  x = crd_x;
  y = crd_y;
  z = crd_z;
}

// Bu metod, bir setter metodudur. Sınıfın X verisi gizli olduğu için X verisini değiştirmek için kullanılır.
// new_x: Eski X koordinatının yerine geçecek olan yeni X koordinatı

void coordinate::set_x(double new_x) { x = new_x; }

// Bu metod, bir setter metodudur. Sınıfın Y verisi gizli olduğu için Y verisini değiştirmek için kullanılır.
// new_y: Eski Y koordinatının yerine geçecek olan yeni Y koordinatı

void coordinate::set_y(double new_y) { y = new_y; }

// Bu metod, bir setter metodudur. Sınıfın Z verisi gizli olduğu için Z verisini değiştirmek için kullanılır.
// new_z: Eski Z koordinatının yerine geçecek olan yeni Z koordinatı

void coordinate::set_z(double new_z) { z = new_z; }

// Bu metod, bir getter metodudur. Sınıfın X verisi gizli olduğu için X verisini elde etmek için kullanılır.

double coordinate::get_x() { return x; }

// Bu metod, bir getter metodudur. Sınıfın Y verisi gizli olduğu için Y verisini elde etmek için kullanılır.

double coordinate::get_y() { return y; }

// Bu metod, bir getter metodudur. Sınıfın Z verisi gizli olduğu için Z verisini elde etmek için kullanılır.

double coordinate::get_z() { return z; }

// Bu metod, 3 boyutlu uzayda 2 nokta arasındaki uzaklığı hesaplar ve döndürür.
// second_cord: Metodu çağıran obje ile arasındaki uzaklık hesaplanacak olan nokta
// sqrt(): Verilen değişkenin karekökünü alır (squareroot). <cmath> kütüphanesinde bulunur.

double coordinate::calculate_dist(coordinate second_cord) {
  double x_dist = second_cord.get_x() - x;  // İki noktanın X koordinatları arasındaki uzaklık hesaplanır.
  double y_dist = second_cord.get_y() - y;  // İki noktanın Y koordinatları arasındaki uzaklık hesaplanır.
  double z_dist = second_cord.get_z() - z;  // İki noktanın Z koordinatları arasındaki uzaklık hesaplanır.

  return sqrt(x_dist * x_dist + y_dist * y_dist +
              z_dist * z_dist);  // İki kere pisagor teoremi uygulanarak uzaklık bulunmuş olur.
}

/* Bu metod, metodu çağıran noktanın X,Y ve Z koordinatlarını verilen katsayı ile çarparak yeni oluşan noktayı döndürür
(Metodu çağıran noktanın X,Y ve Z koordinatlarını değiştirmez!). Vektörler, başlangıç noktası (0,0,0) kabul edilirse,
tek bir nokta ile tanımlanabilir. BU metod aslında vektörlerin ölçeklendirme işlemleri için tanımlanmıştır. coordinate
sınıfı, kod içinde aynı zamanda vektörlerin gösterimi için de kullanılmıştır.*/
// factor: Çarpımda kullanılacak katsayı

coordinate coordinate::crd_multiplication(double factor) {
  double new_x = factor * x;
  double new_y = factor * y;
  double new_z = factor * z;

  return coordinate(new_x, new_y, new_z);
}

/*Bu metod, coordinate sınıfı için yazılmış bir operator overload örneğidir. '+' işaretini bu sınıf için yeniden
tanımlar. Bu tanım yapıldıktan sonra + sembolü coordinate sınıfına ait objeler arasında kullanılabilir. Diyelim ki
coordinate sınıfına ait 2 obje olsun (coordinate_1 ve coordinate_2). Bu durumda bu metodun tanımı yapılınca, derleyici
coordinate_1 + coordinate_2 ifadesini coordinate_1.operator+(coordinate_2) şeklinde okuyacaktır. Metodun amacı metodu
çağıran noktanın X,Y ve Z koordinatları ile parametre noktanın X,Y ve koordinatlarını toplayıp, yeni bir koordinat
döndürmektir.*/
// to_be_added: Toplamada kullanılacak nokta

coordinate coordinate::operator+(coordinate to_be_added) {
  double new_x =
      (this->get_x()) + to_be_added.get_x();  // 'this' pointer ı her zaman metodu çağıran objenin adresini tutar.
  double new_y = (this->get_y()) +
                 to_be_added.get_y();  // Bu durumda this->get_x(), metodu çağıran objenin X koordinatını döndürecektir.
  double new_z = (this->get_z()) + to_be_added.get_z();

  return coordinate(new_x, new_y, new_z);
}

/*Bu fonksiyon, eğer verilen koordinatlar başlangıç noktaları (0,0,0) olan iki vektörün uç noktaları olarak düşünülerse,
iki vektörün skaler çarpımının sonucunu döndürür.*/
// one: Skaler çarpım işlemi için birinci vektörün uç noktası
// two: Skaler çarpım işlemi için ikinci vektörün uç noktası

double dot_product(coordinate one, coordinate two) {
  double result = 0.0;  // Skaler çarpım sonucu ilk başta 0 yapılır.

  result += one.get_x() * two.get_x();  // İki vektörün bitiş noktalarının X koordinatları çarpılıp sonuca eklenir.
  result += one.get_y() * two.get_y();  // İki vektörün bitiş noktalarının Y koordinatları çarpılıp sonuca eklenir.
  result += one.get_z() * two.get_z();  // İki vektörün bitiş noktalarının Z koordinatları çarpılıp sonuca eklenir.

  return result;  // Sonuç döndürülür.
}

/*Bu fonksiyon, eğer verilen koordinatlar başlangıç noktaları (0,0,0) olan iki vektörün uç noktaları olarak düşünülerse,
iki vektörün arasındaki dar açıyı bulur. https://planetcalc.com/8249/ adresinde iki vektör arasındaki açının formülü
verilmiştir.*/
// one: Başlangıç noktası (0,0,0) ise, arasındaki açı bulunacak vektörlerden birincisinin uç noktası
// two: Başlangıç noktası (0,0,0) ise, arasındaki açı bulunacak vektörlerden ikincisinin uç noktası
// acos(): Verilen uzunluğun arkkosinüsünü bulur. <cmath> kütüphanesinde tanımlanmıştır.

double angle_of_vectors(coordinate one, coordinate two) {
  coordinate base_coord = coordinate(0, 0, 0);

  double magnitude_one = one.calculate_dist(base_coord);  // Birinci vektörün uzunluğu hesaplanır.
  double magnitude_two = two.calculate_dist(base_coord);  // İkinci vektörün uzunluğu hesaplanır.

  double angle = (acos(dot_product(one, two) / (magnitude_one * magnitude_two)) * 180) /
                 PI;  // Formül sonucunda vektörler arasındaki açı bulunur. Açı dereceye çevrilir.

  return angle;  // Açı döndürülür.
}

/*Bu fonksiyon, verilen 3 noktaya göre oluşan üçgende, ilk noktanın açısını kosinüs teoremi ile hesaplar.*/
// center: Açısı hesaplanacak nokta (Açıyı oluşturan üçgenin noktalarından üçüncüsü)
// one: Açıyı oluşturan üçgenin noktalarından birincisi
// two: Açıyı oluşturan üçgenin noktalarından ikincisi

double cosinus_theorem(coordinate center, coordinate one, coordinate two) {
  double a = center.calculate_dist(one);
  double b = center.calculate_dist(two);
  double c = one.calculate_dist(two);

  double my_angle = (acos((a * a + b * b - c * c) / (2 * a * b)) * 180) / PI;

  return my_angle;
}

/*Bu fonksiyon, eklem konumlarına göre yönlü açı bulur. Bir eklem açısını oluşturan 3 eklem vardır. Fonsiyon, önce 3
eklemin oluşturduğu üçgenden kosinüs teoremi ile ortanca eklemin açısını bulur ancak bu açı, yanlış açı olabilir. Çünkü
bütün açılar için ortak yön olarak 0 pozisyonunda X-Y düzlemine bakan yön seçildi ve bulunan açı bu yöndeki açı değil
ise, 360'dan çıkarılması gerekebilir. Bu durumda yönlü açı bulmak için vektörel çarpım kullanılabilir. 3 eklemden orta
eklemden bir önceki ekleme giden vektör A, orta eklemden bir sonraki ekleme giden vektör B ise AxB işleminin sonuç
vektörünün Y bileşenine göre (vektörün X bileşeninin işareti de önemli) açı yönü bulunabilir.*/
// center: Ortanca eklem noktası
// before: Önceki eklem noktası
// next: Sonraki eklem noktası
// joint_1_angle: 1. eklem açısı

double find_angle(coordinate center, coordinate before, coordinate next, double joint_1_angle) {
  double my_angle = cosinus_theorem(
      center, before,
      next);  // Kosinüs teoremi ile ortanca eklemin açısı hesaplanır (Kontrol edilmedi. Yanlış olabilir.).

  coordinate new_before = coordinate(
      before.get_x() - center.get_x(), before.get_y() - center.get_y(),
      before.get_z() -
          center.get_z());  // Ortanca eklemin (0,0,0)'a taşındığı düşünülürse, önceki eklemin yeni konumu hesaplanır.
  coordinate new_next = coordinate(
      next.get_x() - center.get_x(), next.get_y() - center.get_y(),
      next.get_z() -
          center.get_z());  // Ortanca eklemin (0,0,0)'a taşındığı düşünülürse, sonraki eklemin yeni konumu hesaplanır.
                            // (Vektör çarpımı yapmak için ortanca eklem (0,0,0) noktasına taşınmalı.)

  double cross_product_y = new_before.get_z() * new_next.get_x() -
                           new_before.get_x() * new_next.get_z();  // AxB vektörünün Y bileşeni hesaplanır.

  if (((joint_1_angle < 90) && (joint_1_angle > -90)) && cross_product_y > 0)
    my_angle = 360 - my_angle;  // Robot kol, +X yarım küresine bakıyorsa ve vektörel çarpımın Y bileşeni 0 dan büyükse,
                                // açı yanlıştır ve 360'dan çıkarılır.
  if (((joint_1_angle > 90) || (joint_1_angle < -90)) && cross_product_y < 0)
    my_angle = 360 - my_angle;  // Robot kol, -X yarım küresine bakıyorsa ve vektörel çarpımın Y bileşeni 0 dan küçükse,
                                // açı yanlıştır ve 360'dan çıkarılır.

  return my_angle;  // Açı döndürülür.
}

/*Bu fonksiyon, rotasyon matrisi formülleri kullanarak 3 boyutlu uzaydaki bir noktayı XY düzlemine göre verilen açı
kadar döndürür, noktanın konumunu günceller. 1. eklem açısı hesaplandıktan sonra diğer eklemler bu açı kadar döndüğü
için eklemlerin yeni konumlarını bulmak için kullanılır.*/
// to_be_rotated: Döndürülecek noktanın referansı (Referans olduğu için yapılan değişim main() deki noktayı
// etkileyecektir.) angle: Noktanın döndürüleceği açı miktarı sin()/cos(): Sinüs ve kosinüs hesaplamak için kullanılan
// fonksiyonlar (<cmath> kütüphanesinde tanımlanmışlardır.).

void rotate_on_xy(coordinate& to_be_rotated, double angle) {
  double angle_radian = (angle * PI) / 180;

  double new_x =
      to_be_rotated.get_x() * cos(angle_radian) - to_be_rotated.get_y() * sin(angle_radian);  // Rotation matrix is used
  double new_y = to_be_rotated.get_x() * sin(angle_radian) + to_be_rotated.get_y() * cos(angle_radian);

  to_be_rotated.set_x(new_x);
  to_be_rotated.set_y(new_y);
}

/*Bu sınıf, f310 dan gelen mesajları bir arada tutmak için tanımlanmıştır.*/

class joy_message {
  double axes[6];
  double buttons[12];

 public:
  void set_axes(double, double, double, double, double, double);
  double get_axis(int);
  void set_buttons(double, double, double, double, double, double, double, double, double, double, double, double);
  double get_button(int);
};

/*Bu metod ile tek seferde bütün axis değerleri güncellenir*/

void joy_message::set_axes(double new_axis_0, double new_axis_1, double new_axis_2, double new_axis_3,
                           double new_axis_4, double new_axis_5) {
  axes[0] = new_axis_0;
  axes[1] = new_axis_1;
  axes[2] = new_axis_2;
  axes[3] = new_axis_3;
  axes[4] = new_axis_4;
  axes[5] = new_axis_5;
}

/*Bu metod ile herhangi bir axis değerine index'i ile ulaşılabilir.*/

double joy_message::get_axis(int axis_index) { return axes[axis_index]; }

/*Bu metod ile tek seferde bütün buton değerleri güncellenir*/

void joy_message::set_buttons(double new_button_0, double new_button_1, double new_button_2, double new_button_3,
                              double new_button_4, double new_button_5, double new_button_6, double new_button_7,
                              double new_button_8, double new_button_9, double new_button_10, double new_button_11) {
  buttons[0] = new_button_0;
  buttons[1] = new_button_1;
  buttons[2] = new_button_2;
  buttons[3] = new_button_3;
  buttons[4] = new_button_4;
  buttons[5] = new_button_5;
  buttons[6] = new_button_6;
  buttons[7] = new_button_7;
  buttons[8] = new_button_8;
  buttons[9] = new_button_9;
  buttons[10] = new_button_10;
  buttons[11] = new_button_11;
}

/*Bu metod ile herhangi bir buton değerine index'i ile ulaşılabilir.*/

double joy_message::get_button(int button_index) { return buttons[button_index]; }

/*Fabrik algoritması, verilen eklemlerin konumlarını, robot kolun uç noktası new_end_point_pos olacak şekilde yeniden
hesaplar. Algoritmanın çalışma mantığı için: https://drive.google.com/drive/folders/10Jx3R2WmaSRwj1mYTVzVz-0sIhBbL4JL*/
// my_joints: Eski eklem konumlarını içeren vektörün referansı (Referans olduğu için eklem konumları kolaylıkla
// güncellenebilir.). link_lengths: Link uzunluklarını tutan array'in ilk elemanının adresi new_end_point_pos: Robot
// kolun ucunun gitmesinin istendiği nokta REACH: Robot kolun tam açılımda uzanabildiği maksimum mesafe

void FABRIK_algorithm(vector<coordinate>& my_joints, double* link_lengths, coordinate new_end_point_pos, double REACH) {
  int i;
  double r, lambda;
  double distance_from_beginning = abs_val(new_end_point_pos.calculate_dist(my_joints[0]));

  if (distance_from_beginning > REACH) {
    for (i = 0; i < my_joints.size(); i++) {
      r = abs_val(new_end_point_pos.calculate_dist(my_joints[i]));
      lambda = link_lengths[i] / r;

      my_joints[i + 1] = my_joints[i].crd_multiplication(1 - lambda) + new_end_point_pos.crd_multiplication(lambda);
    }

  }

  else {
    coordinate b;
    b = my_joints[0];

    double distance_to_target = new_end_point_pos.calculate_dist(my_joints[my_joints.size() - 1]);

    while (distance_to_target > TOLERANCE) {
      my_joints[my_joints.size() - 1] = new_end_point_pos;

      for (i = my_joints.size() - 2; i >= 0; i--) {
        r = abs_val(my_joints[i + 1].calculate_dist(my_joints[i]));
        lambda = link_lengths[i] / r;

        my_joints[i] = my_joints[i + 1].crd_multiplication(1 - lambda) + my_joints[i].crd_multiplication(lambda);
      }

      my_joints[0] = b;

      for (i = 0; i < my_joints.size() - 1; i++) {
        r = abs_val(my_joints[i + 1].calculate_dist(my_joints[i]));
        lambda = link_lengths[i] / r;

        my_joints[i + 1] = my_joints[i].crd_multiplication(1 - lambda) + my_joints[i + 1].crd_multiplication(lambda);
      }

      distance_to_target = abs_val(my_joints[my_joints.size() - 1].calculate_dist(new_end_point_pos));
    }
  }
}

/* Bu fonksiyon, eklem açıları doğru mu değil mi onu kontrol eder */
// mode: fabrik_v2 için 2, fabrik_v4 için 4
// joint_angles: Eklem açılarını içeren double array'inin ilk elemanının adresi

bool check_angles(int mode, double* joint_angles) {
  bool angles_valid = true;

  if ((joint_angles[0] < J1_ANGLE_BORDER_MIN) || (joint_angles[0] > J1_ANGLE_BORDER_MAX)) {
    angles_valid = false;
    return angles_valid;
  }

  if ((joint_angles[1] < J2_ANGLE_BORDER_MIN) || (joint_angles[1] > J2_ANGLE_BORDER_MAX)) {
    angles_valid = false;
    return angles_valid;
  }

  if ((joint_angles[2] < J3_ANGLE_BORDER_MIN) || (joint_angles[2] > J3_ANGLE_BORDER_MAX)) {
    angles_valid = false;
    return angles_valid;
  }

  if (mode == 2) {
    if ((joint_angles[3] < J4_ANGLE_BORDER_MIN) || (joint_angles[3] > J4_ANGLE_BORDER_MAX)) {
      angles_valid = false;
      return angles_valid;
    }

    if ((joint_angles[4] < J5_ANGLE_BORDER_MIN) || (joint_angles[4] > J5_ANGLE_BORDER_MAX)) {
      angles_valid = false;
      return angles_valid;
    }
  }

  else {
    if (((joint_angles[3] * 180) / PI < J4_ANGLE_BORDER_MIN) || ((joint_angles[3] * 180) / PI > J4_ANGLE_BORDER_MAX)) {
      angles_valid = false;
      return angles_valid;
    }

    if (((joint_angles[4] * 180) / PI < J5_ANGLE_BORDER_MIN) || ((joint_angles[4] * 180) / PI > J5_ANGLE_BORDER_MAX)) {
      angles_valid = false;
      return angles_valid;
    }
  }

  return angles_valid;
}
