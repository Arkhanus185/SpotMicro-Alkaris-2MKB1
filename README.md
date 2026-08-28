# Alkaris 2MKB1

**Special Thanks / Acknowledgements:**
A massive thank you to [advancedhobbylab](https://github.com/advancedhobbylab/spotmicro) for providing the foundational inverse kinematics and locomotion framework. Without their excellent open-source contribution, the advanced AI, Target tracking, and obstacle avoidance features of Alkaris would not have been possible.

Oww Yeee
---

## Proje Hakkinda

Alkaris, Raspberry Pi 4 uzerinde calisan, yapay zeka destekli otonom bir robot kopek projesidir. Bu proje, sadece bir robotu hareket ettirmeyi degil, ayni zamanda goruntu isleme ve buyuk dil modelleri (Gemini AI) kullanarak robota "gorme, anlama ve takip etme" yetenekleri kazandirmayi amaclar.

Bu rehber, eline ilk defa Raspberry Pi alan bir universite ogrencisi dusunulerek hazirlanmistir. Asagidaki adimlari sirasiyla takip ederek sistemi sorunsuz bir sekilde kurabilirsiniz. Neyi neden kullandığımıza dair daha fazla detay isterseniz Alkaris_Makale_Github.pdf dosyasını inceleyebilirsiniz.

## Kurulum Rehberi

### Adim 1: Raspberry Pi Guncellemeleri
Daima en guncel sistemle baslamak hatalari onler. Raspberry Pi terminalini acin ve su komutlari sirasiyla girin:

`sudo apt update`
`sudo apt upgrade`

### Adim 2: Projeyi Bilgisayara Indirme
Bu projedeki tum kodlari cihazina indirmek icin Git aracini kullanacagiz:

`git clone https://github.com/Arkhanus185/SpotMicro-Alkaris-2MKB1.git`
`cd SpotMicro-Alkaris-2MKB1`

### Adim 3: Sanal Ortam (Virtual Environment) Kurulumu
Projelerinizdeki Python kutuphanelerinin birbirine karismamasi icin "sanal ortam" kullanmak en profesyonel yontemdir. Bu, projeniz icin yalitilmis bir alan yaratir.

Sanal ortami olusturmak icin:
`python3 -m venv alkaris_env`

Sanal ortami aktif hale getirmek icin:
`source alkaris_env/bin/activate`

Not: Bu komutu girdikten sonra terminalde isminizin basinda (alkaris_env) yazisini gormelisiniz. Sistemi her yeniden baslattiginizda projeyi calistirmadan once bu "source" komutunu tekrar girerek sanal ortami aktif etmeniz gerekir.

### Adim 4: Gerekli Kutuphanelerin Yuklenmesi
Sanal ortamimiz aktifken, robotun gorme, duyma ve dusunme yetenekleri icin gerekli olan paketleri tek bir komutla kuralim:

`pip install -r requirements.txt`

### Adim 5: Yapay Zeka Anahtari ve Denge Kalibrasyonu
Sistemin calismasi icin yapmaniz gereken bazi kucuk ayarlar var.

1. Gemini API Anahtari: 
`locomotion_final_stream.py` dosyasini bir metin editoru ile acin. `GEMINI_API_KEY` yazan degiskenin icerisine kendi Google Gemini API anahtarinizi tirnak isaretleri icinde yapistirin ve kaydedin.

2. Denge Kalibrasyonu (MPU6050):
Robotunuzun duzgun yuruyebilmesi icin denge sensorunun nerenin "duz" oldugunu ogrenmesi gerekir. Robotunuzu duz bir zemine koyun, hic hareket ettirmeyin ve su komutu calistirin:
`python denge_kalibrasyon.py`
Islem bittiginde kalibrasyon verileriniz otomatik olarak kaydedilecektir.

### Adim 6: Motor (Servo) Kalibrasyonu ve Cift PCA9685 Kullanimi
Robotumuzda toplam 12 adet servo motor bulunmaktadir. Tek bir PCA9685 motor surucu karti 16 motora kadar desteklese de, bu projede 2 adet PCA9685 karti kullanilmistir. Bunun amaci on ve arka bacaklarin kablo karmasasini onlemek ve motorlarin cektigi yuksek akimi iki ayri karta bolusturerek asiri isinmayi engellemektir. On kartin I2C adresi 0x40, arka kartin adresi 0x41 olarak ayarlanmistir.

Robotu birlestirmeden once motorlarin 90 derece (merkez) acisina getirilmesi cok onemlidir:
1. `python offset.py 0` komutunu calistirarak 0. porttaki motoru merkez aciya getirin.
2. Eger motor tam 90 derecede degilse, terminalden eksi (-) veya arti (+) degerler girerek motorun tam dik aciya gelmesini saglayin ve cikan sapma (offset) degerini not edin.
3. Bu islemi tum motorlar icin yapin.
4. Ana dizinde `config.csv` adinda bir dosya olusturun ve buldugunuz 12 adet offset degerini aralarinda virgul olacak sekilde yan yana yazip kaydedin (Ornek: 0, -15, 12, 0, ...). Program yuruyus esnasinda bu hatalari otomatik olarak telafi edecektir.

### Adim 7: Sistemi Baslatma ve Web Arayuzune Erisim
Her sey hazir. Ana otonom programi calistirmak icin terminale su komutu girin:
`python locomotion_final_stream.py`

Terminalde sistemin hazir olduguna dair yazilari gordugunuzde, ayni agdaki bilgisayarinizin veya telefonunuzun internet tarayicisini acin. Adres cubuguna Raspberry Pi cihazinizin IP adresini ve sonuna 5000 portunu yazin (Ornek: http://192.168.1.50:5000).

Karsiniza cikan web arayuzunden:
- Robotun kamerasindan canli goruntu alabilir,
- Manuel yuruyus ve egilme komutlari verebilir,
- Hedef Takip veya Otonom Engelden Kacma modlarini tek tikla aktif edebilirsiniz.

Iyi calismalar dilerim.
