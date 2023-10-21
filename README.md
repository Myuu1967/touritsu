初めに”DHA_Pendulum”というフォルダーを作って下さい。
$ mkdir DHA_Pendulum

CMakeLists.txt：CMake用のファイルです。（フォルダー”DHA_Pendulum”の中に移す）
DHA_Pendulum.c：この倒立振子用のCファイルです。（フォルダー”DHA_Pendulum”の中に移す）
DHA_Pendulum.pdf：この倒立振子の回路図です。
DHA_Pendulum.uf2：実行可能ファイルです。Pico内に転送して利用します。
_CMakeLists.txt：CMake用のファイルです。CMakeLists.txtをフォルダー”DHA_Pendulum”に移した後に、CMakeLists.txtとリネームする
README.md:このファイルです。

ビルドの仕方：
(1)フォルダーDHA_Pendulumの中で下のように打つ。（”$”は打たない）
$ mkdir build
$ cd build
（２）フォルダがbuildに移るので、次のように打つ。
$ cmake ../..
$ cd DHA_Pendulum
$ make -j4

以上です
