# SPRAWOZDANIE z laboratorium nr 4
* Piotr Patek 324879
* Kacper Bielak 324852

# Zadanie 1
Analiza struktury systemu robota TIAGo (rviz,rqt_tf_tree,rqt_graph,narzędzia: topic, node, doctor-report).

Uruchomiono system robota TIAGo z wykorzystaniem komendy
```bash
ros2 launch hello_moveit stero_tiago_gazebo.launch.py is_publi_sim:=True navigation:=True moveit:=True world_name:=stero
```

Jak można zauważyć wykorzystano plik uruchomieniowy `stero_tiago_gazebo.launch.py` stworzony w ramach pakietu `hello_moveit` w trakcie bloku manipulacyjnego.
## Rviz
![Alt text](images/rviz/ZADANIE1_4.jpg)

Pierwszym z badanych narzędzi był program do wizualizacji systemu - `RViz`. Służy on do reprezentacji obecnego stanu robota i tego jak postrzega on środowisko w którym się znajduje. Jak można zauważyć na rysunku widoczna jest mapa środowiska, w której przeszkody reprezentowane są jako czarne elementy. Dodatkowo widoczna jest chmura czerwonych punktów stworzona przez LIDAR 2D umieszczony w bazie robota. Dzięki niemu robot wie, że oprócz ścian o których istnieniu wiedział już z mapy, w świecie znajdują się także inne przeszkody takie jak stół, którego nogi widoczne są na rysunku.

Oprócz tego w programie możliwa jest modyfikacja widoków po lewej stronie (np. włączenie widoku TF, włączenie markerów) oraz zmiana orientacji kamery po rozwinięciu menu po prawej stronie.

## rqt_tf_tree
![Alt text](images/rqt_tf_tree/ZADANIE1_2_4.jpg)
![Alt text](images/rqt_tf_tree/ZADANIE1_2_2_4.jpg)
![Alt text](images/rqt_tf_tree/ZADANIE1_2_3_4.jpg)
![Alt text](images/rqt_tf_tree/ZADANIE1_2_4_4.jpg)
![Alt text](images/rqt_tf_tree/ZADANIE1_2_5_4.jpg)
![Alt text](images/rqt_tf_tree/ZADANIE1_2_6_4.jpg)

W następnym kroku skupiono się na drzewie transformacji TF, które wyświetlono z wykorzystaniem narzędzia `rqt_tf_tree`. Jak widać na rysunkach drzewo przekształceń jest bardzo obszerne. Jego korzeń znajduje się w układzie `map` związanym z mapą świata. Następnie widoczne są przekształcenia, aż do układu `base_link`, który jest glównym układem związanym z samym robotem. Bezpośrednio do niego przyłączone jest wiele układów, z czego większość to układy związane z różnymi czujnikami takimi jak mikrofony, sonary, LIDAR czy jednostka inercyjna. Po lewej stronie grafu widoczna jest dalsza część drzewa przekształceń zawierająca układ korpusu, który ostatecznie rodziela się na graf układów ramienia oraz układy związane z głową robota.

To co jest także istotne to fakt, że dla każdego układu na grafie możemy odczytać takie informacje jak częstotliwośc publikacji współrzędnych układu, wielkość bufora zawierającego jego poprzednie wartości czy nazwę elementu publikującego dany układ.

## rqt_graph
![Alt text](images/rqt_graph/ZADANIE1_5.jpg)
![Alt text](images/rqt_graph/ZADANIE1_8_5.jpg)
![Alt text](images/rqt_graph/ZADANIE1_3_5.jpg)
![Alt text](images/rqt_graph/ZADANIE1_4_5.jpg)
![Alt text](images/rqt_graph/ZADANIE1_5_5.jpg)
![Alt text](images/rqt_graph/ZADANIE1_6_5.jpg)
![Alt text](images/rqt_graph/ZADANIE1_7_5.jpg)

Po uruchomieniu programu `rqt_graph` wyświetlony został graf węzłów występujących w badanym systemie wraz z połączeniami między nimi. Jak można zauważyć rozważany system jest bardzo rozbudowany. Widoczne są węzły odpowiedzialne zarówno za samo sterowanie bazą mobilną - `mobile_base_controller` oraz węzły będące elementami systemu nawigacji - `bt_navigator`, `velocity_smoother`, `waypoint_follower` czy `global_costmap`.

Oprócz tego widoczne są także węzły odpowiedzialne za przełączanie pomiędzy różnymi strumieniami sterowania bazą mobilną, węzły czujników czy węzły systemów transformacji.

Z racji, że badany system działa w środowisku symulowanym nie mogło także zabraknąć elementu związanego z symulacją. W tym przypadku z lewej strony grafu widać węzeł `gazebo`.

## Narzędzia:
### Topic
![Alt text](images/ros2_topic/AZADANIE1_6.jpg)
![Alt text](images/ros2_topic/AZADANIE1_2_6.jpg)
![Alt text](images/ros2_topic/AZADANIE1_3_6.jpg)
![Alt text](images/ros2_topic/AZADANIE1_4_6.jpg)

Listę tematów, które wcześniej ukazane były jako połączenia między węzłami na grafie `rqt_graph` można wyświetlić poleceniem `ros2 topic list`. Ukazuje się nam wtedy bardzo długa lista tematów, z których można wyróżnić m.in.:
- `/cmd_vel` - zadawanie prędkości bazy mobilnej
- `/base_imu` - orientacja robota
- `/scan_raw` - chmura punktów generowana przez LIDAR
- `/global_costmap/costmap` - mapa kosztów
- `/tf` - transformacje
- `/clock` - zegar symulacji

Lista pokazuje przepływ danych między węzłami odpowiedzialnymi za sterowanie, percepcję, planowanie i diagnostykę robota.
### Node
![Alt text](images/ros2_node/BZADANIE1_7.jpg)
![Alt text](images/ros2_node/BZADANIE1_2_7.jpg)
![Alt text](images/ros2_node/BZADANIE1_3_7.jpg)
![Alt text](images/ros2_node/BZADANIE1_4_7.jpg)

Komenda `ros2 node list` umożliwia wyświetlenie listy węzłów występujących w badanych systemie. Wyróżnić możemy m.in.:

- `gazebo` - symulacja
- `bt_navigator`, `amcl` - nawigacja
- `base_imu`, `base_laser` `base_sonar0*` - czujniki
- `head_controller`, `mobile_base_controller`, `arm_controller` - kontrolery sprzętowe

### Doctor-report
![Alt text](images/doctor/DOCTOR.jpg)

Polecenie doctor --report generuje szczegółowy raport diagnostyczny, zawierający konfigurację sieci (adresy IP, maski, MAC, MTU) oraz informacje o systemie operacyjnym (wersja jądra, platforma). Służy do diagnozy i weryfikacji środowiska.
# Zadanie 2 - Identyfikacja tematów sterowania i sensorów robota

## Sterowanie bazą robota
### `mobile_base_controller`
![Alt text](images/control/mobile_base.jpg)

Sterowanie bazą robota w badanym systemie TIAGo odbywa się za pośrednictwem kontrolera `mobile_base_controller`. Subskrybuje tematy takie jak `/cmd_vel_unstamped` i `/clock`. Publikuje dane odometrii `/odom` oraz  transformacje `/tf`. Udostępnia serwisy zarządzania parametrami, takie jak ich opisywanie, pobieranie i ustawianie. Węzeł ten integruje sterowanie ruchem z systemem nawigacji i planowania.


### Sterowanie prędkością bazy
![Alt text](images/control/PREDKOSC.jpg)

Sterowanie prędkością bazy mobilnej może odbywać się w różny sposób. Prędkość może być publikowana bezpośrednio na `/cmd_vel`, może pochodzić z joysticka - `/joy_vel`, klawiatury - `/key_vel` czy systemu nawigacji - `/cmd_vel_nav`. Przełączanie pomiędzy tymi tematami odbywa się z wykorzystaniem multipleksera zlokalizowanego w węźle `twist_mux`. Ostatecznie prędkość przekazywana jest do wcześniej omawianego kontrolera `/mobile_base_controller`.

## Odometria
![alt text](images/control/ODOMETRIA1.jpg)

Odometria robota publikowana jest przez węzeł `/mobile_base_controller` na temat `/mobile_base_controller/odom`. Jest to odometria obliczona przez samego robota w symulowanym środowisku. Oprócz tego dostępny jest także temat `/groud_truth_odom`, na którym publikowana jest odometria powstała na podstawie pozycji robota wyciągniętej bezpośrendio z symulatora.


## Czujnik LiDAR
![alt text](images/control/LIDAR.jpg)

Węzeł `/base_laser` w ROS2 odpowiada za obsługę danych z LIDAR-u. Subskrybuje tematy takie jak /clock i /parameter_events, publikuje dane skanowania laserowego `/scan_raw` oraz udostępnia serwisy zarządzania parametrami (opis, pobieranie, ustawianie). Obsługuje skanowanie otoczenia, wspierając nawigację i unikanie przeszkód.

## Kamery RGB-D
![alt text](images/control/RGB-D.jpg)

Robot TIAGo posiada także elementy wizyjne takie jak kamery RGB-D. Dane z niej publikowane są na temacie `/head_front_camera/rgb/image_raw` oraz `/head_front_camera/depth_registered/image_raw`. Na zdjęciu powyżej przedstawiono obraz z kamery głębi wyświetlony w wizualizacji w programie RViz.

# Zadanie 3
Tworzenie pierwszego węzła do testu przestrzeni pracy:
TODO://
KOD, OPIS KODU, OPIS ZDJEC, ZDJECIA
