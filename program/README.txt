Plik bez polskich znakow na wszelki wypadek jakby ktos mial jakies dziwne kodowanie.

INSTRUKCJA DLA LINUX UBUNTU+ KUBUNTU, dla innych powstana moze pozniej

1. Ze strony www.gazebosim.org/documentation.html nalezy pobrac zrodla
gazebo-1.4.0.
2. Wypakowac do jakiegos katalogu
3. Wykonac polecenie: sudo apt-get install build-essential libtinyxml-dev libtbb-dev libxml2-dev libqt4-dev pkg-config  libprotoc-dev libfreeimage-dev libprotobuf-dev protobuf-compiler libboost-all-dev freeglut3-dev cmake libogre-dev libtar-dev libcurl4-openssl-dev libcegui-mk2-dev
4. Wejsc do katalogu, w ktorym sa wypakowane zrodla
5. Wykonac polecenie: mkdir build
6. Wykonac polecenie: cd build
7. Wykonac polecenie: cmake ../
8. Jesli polecenie zakonczy sie sukcesem to: make (lub make -jX gdzie X to liczba rdzeni +1)
9. Wykonac: make install (byc moze jako root)
10 Wykonac: echo "source /usr/local/share/gazebo/setup.sh" >> ~/.bashrc
11.Ostatczenie: source ~/.bashrc
WIECEJ INFO ODNOSNIE INSTALACJI ORAZ EWENTUALNYCH PROBLEMOW POD ADRESEM:
http://gazebosim.org/wiki/1.4/install



EDYCJA SCIEZEK BEZWZGLEDNYCH
1. w katalogu urytym .gazebo, ktory powinien zostac umieszczony w $HOME nalezy
wyedytowac models/jurek/model.sdf i na koncu pliku zmienic sciezke bezwzgledna z /home/kuba
na swoj katalog domowy
2. Plik zrodlowy znajdujacy sie w program/my_plugin/my_plugin.cc nalezy wyedytowac
i zamienic sciezke bezwgledna (CTRL +F /home/kuba) na wlasna.



URUCHOMIENIE PROGRAMU

1. Program nalezy skompilowac, w tym celu nalezy usunac z my_plugin katalog
build i utworzyc ponownie
2. Z poziomu katalogu build (cd build) nalezy wykonac: cmake ../
3. wykonac: make
4. Powinno sie skompilowac :)
5. Gazebo nalezy uruchomic podajac plik xml: gazebo /sciezka/do/pliku/MojSwiat.xml
6. Po uruchomieniu programu nalezy dodac model do widoku (INSERT->Jurek)

SAMOCHOD powinien jechac a zrzuty z kamery powinny pojawic sie w wybranym
katalogu


BLEDY:
Przy zapisie zrzutow predkosc symulacji spada do 0.2 (pracujemy nad tym)


KONTAKT:
kuba.g4 na gmail.com
pawelbogner na gmail.com