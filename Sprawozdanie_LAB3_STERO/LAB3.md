# Zadanie 1
Zadanie 1 zostało wykonane pomyślnie. Utworzono przestrzeń pracy tiago na podkładce `/opt/ros/iron` oraz przestrzeń pracy stero na podkładce tiago. W przestrzeni pracy stero znajdują się pakiety utworzone w ramach laboratorium 1 i 2, zgodnie z wymaganiami.


# Zadanie 2
Węzeł **pick_place** został napisany na podstawie węzła **one_grasp** z laboratorium 2. Węzeł realizuje funkcję chwytania obiektu green_cube_3, unosi go na wysokość 5 cm nad stół i odkłada w inne miejsce, zachowując odwróconą współrzędną Y, aby osiągnąć przesunięcie bez zmiany orientacji. W przypadku, gdy chwytanie obiektu jest niemożliwe, węzeł wyświetla odpowiedni komunikat i kończy działanie bez błędu.

# Zadanie 3
Wynik zadania został zaprezentowany prowadzącemu laboratoria, który podtwierdził wykonanie zadania. Zdjęcia przedstawiają poprawne działanie kodu oraz błąd w sytuacji błędnego ustawienia kostki:
![Alt text](zad3_3.png)
# Zadanie 4
Dla obiektu green_cube_3 wyznaczono 12 chwytów z jednej ścianki sześcianu, korzystając z obrotów układu obiektu.

Pozycje `grasp` i `pregrasp` wyznaczane są w następujący sposób:

$T^B_{E,gr} = T^B_O T^O_{Ogr} T^O_{A7,gr} T^{A7,gr}_{E,gr}$

$T^B_{E,pre} = T^B_O T^O_{Ogr} T^O_{A7,gr}  T^{A7,gr}_{A7,pre} T^{A7,pre}_{E,pre}$

Dodatkowe przekształcenie $T^O_{Ogr}$ wykorzystywane jest zarówno przy wyznaczaniu pozycji `grasp` jak i `pregrasp`. Składa się ono z rotacji i podrotacji.

Wykorzystywane rotacje: $\text{I}$, $\text{RY}(90 \deg)$, $\text{RY}(180 \deg)$, $\text{RY}(270 \deg)$

Wykorzystywane podrotacje: $\text{I}$, $\text{RZ}(-30 \deg)$, $\text{RZ}(30 \deg)$

$T^O_{Ogr} = \text{ROTATION} \cdot \text{SUBROTATION}$

Obiekty takie jak sześciany pozwalają na wyznaczenie chwytów dla każdej z 6 ścianek(co daje łącznie do 72 chwytów), jednak w raporcie umieściliśmy zdjęcia wszystkich możliwych chwytów dla jednej ścianki zgodnie z poleceniem.
# Zadanie 5
Węzeł **show_side_grasps** został napisany zgodnie z wytycznymi. Węzeł wyświetla za pomocą markerów w RViz oraz metod `publishAxis()` i `publishGrasps()` trzy chwytaki, pokazując każdy chwyt osobno, aż do naciśnięcia przycisku „Next” (lub „Break”, co kończy działanie programu). Dla każdego chwytu wyświetlane są: model chwytaka, układ związany z chwytakiem (dla pozycji chwytu i przed chwytem) oraz układ związany z obiektem.
Zdjęcia ekranu z RViz, które dokumentują każdy chwyt:
![Alt text](combined_image_3x4.png)

# Zadanie 6
 Chwyty zostały wyznaczone i zaprezentowane prowadzącemu.
# Zadanie 7
Węzeł **show_all_grasps** został napisany na podstawie węzła **show_side_grasps**. Węzeł wyświetla wszystkie wyznaczone chwytaki, pokazując łącznie 6 chwytów w RViz. W raporcie umieszczono zrzuty ekranu z RViz, które przedstawiają po jednym chwycie dla każdej ścianki.
![Alt text](combined_image_3x2.png)

# Zadanie 8
Węzeł **show_selected_grasps** został utworzony na podstawie węzła **show_all_grasps**. Węzeł wybiera i wyświetla chwytaki dla obiektu green_cube_3, które mają największą szansę powodzenia, czyli są skierowane „od góry”.
![Alt text](combined_image_vertical.png)

# Zadanie 9
Węzeł **pick_place_robust** został opracowany na podstawie węzła **pick_place**. Węzeł realizuje to samo zadanie co pick_place, ale dodatkowo próbuje chwytania obiektu green_cube_3 kolejnymi chwytami, zgodnie z procedurą opisaną w Zadaniu 7. Dzięki temu jest w stanie podnieść obiekt niezależnie od ścianek, na których leży.

# Zadanie 10
Węzeł **tower_2** został napisany na podstawie węzła **pick_place_robust**. Realizuje on zadanie układania wieży z dwóch klocków: green_cube_2 jako górny element oraz green_cube_3 jako dolny. Węzeł zapewnia poprawne ułożenie tych klocków jeden na drugim.

# Zadanie 11
W zadaniu 11 rozszerzono funkcjonalność węzła **tower_2** do węzła **tower_3**, umożliwiając ułożenie wieży z trzech klocków: green_cube_1, green_cube_2 i green_cube_3. Implementacja wykorzystała rozwiązanie z zadania 10, wprowadzając obsługę dodatkowego klocka, co wymagało uwzględnienia jednej dodatkowej operacji chwytania i ustawiania.
