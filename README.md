# KINGROON KP3+3Dtouch+WiFi+UI mod  
# KINGROON KP3に3DtouchとWiFi機能を追加しUIも少し変更  

**★2023 11/13 追記**  
・初回にアップロードしたMarlinCore.cppにバグがいました。イニシャル時のSDカードのマウント処理が抜けていたのでWiFiモジュールに対してのファームの書き込みやSSID等の設定ができないコードをアップしていました。すみません。現在アップされているファイルは修正済みです。  

![KP3_Overall](https://github.com/Toshi2020/KINGROON-KP3_3Dtouch_WiFi_UI-mod/assets/81674805/7c97fac4-4346-4654-b607-a660e222c16b)
![ScreenShot2](https://github.com/Toshi2020/KINGROON-KP3_3Dtouch_WiFi_UI-mod/assets/81674805/90523231-48f7-4dfd-b506-f601d461fced)

**●概要**  
・KINGROON KP3に3Dtouchによる自動レベリング機能を追加しました。  
・WiFi機能の追加も行いました。KP3に挿してあるSDカードにCuraからWiFiでファイルを転送して印刷の自動開始を行うことができます。  
・合わせてバグ対応やUIを使いやすいように少し変更しています。  
  
・KP3のマザーボードMKS(makerbase) robin mini用にMKSが過去に提供したファームウェアは、同社のWiFiモジュール(MKS robin WiFi)に対応はしているものの3Dtouchには対応していないため、今回はMarlin2.1.2.1をベースに組み込みました。  
・robin miniにはボード上にMKS WiFiモジュールを挿すためのソケットが用意されています。MKS WiFiモジュールとはESP12SというTCP/IPスタックを持つMPUモジュールを、MKSのボード用にピンを出した基板に載せた物です。  
・今のMarlinのソースではMKS WiFiは同社のTFT35というLCDモジュールとセットで組み込むようになっています(WiFiモジュール自体をTFT35に挿す仕組みになっている)。今回はWiFi対応部分のコードをコピーしてTFT35向けのUI部分をごっそり削除した物を組み込んでいます。  
・なおこのモジュールはWiFi接続だけでなくクラウドに接続してファイルを受け取ることもできるのですが、今回はクラウド接続機能はディセイブルにしています。  
  
・3Dtouch取り付け部の3Dプリントデータなどはこちらにアップしました↓。  
https://www.thingiverse.com/thing:6295855  
・関連する話題をブログにアップしました。  
https://minkara.carview.co.jp/userid/3336538/blog/47323023/  

・参考までにrobin mini対応のMKS版のファームウェアはこちらのmks firmwareディレクトリ、回路図等はhardwareディレクトリ以下にあります↓  
https://github.com/makerbase-mks/MKS-Robin-mini  
・MKS WiFiモジュール自体のファームウエア、回路図、通信プロトコルなどはこちら↓  
https://github.com/makerbase-mks/MKS-WIFI  
  
**●ビルドの方法**  
・Marlin公式ページからリリース版2.1.2.1.zipをダウンロードしてPCに展開します。  
https://marlinfw.org/meta/download/  
・今回アップしたMarlinフォルダとplatformio.iniを先ほど展開したフォルダの下に上書きコピーします。  
・PlatformIOを組み込んだVisual Studio Codeでビルドします。  
・.pio\build\mks_robin_mini\Robin_mini.binをSDカードにコピーしてKP3に刺して通電するとアップデートが行われます。  
  
**●3Dtouchの結線**  
・3Dtouchからのケーブルには電源＆制御線の3ピンと、センサ出力の2ピンのコネクタが付いています。(制御はラジコンサーボと同じ要領でパルス幅でピンを出し入れの指示を出します。センサ出力は普段はLですがベッドにタッチするとHのパルスが得られます。3Dtouch側はオープンドレインでマザーボード側でプルアップされています。)  
・制御線側の3ピンをrobin miniの空きコネクタに、センサ出力側の2ピンを空いているZ+コネクタに接続します。マザーボード側のコネクタはいずれも3ピンのうち中央がGNDなので注意が必要です。またGNDを挟んで信号線と反対側のピンは電源ラインなので、センサ出力線を間違ってそちらに接続すると3Dtouchが破損すると思うので注意が必要です。  
・元々製品に付属していたケーブルが少しだけ短いこともあり、私は3ピンのXHコネクタを使って延長ケーブルを作りました。延長ケーブルの先端は絶縁被覆を1cmほど剥いて心線をねじってから半田メッキして3Dtouch側のコネクタに挿入してテープで止めました。  
・従来Z軸のホーミング(原点出し)に使っていたマイクロスイッチはZ-コネクタと接続したままです。3Dtouchが不調になってもファームを焼き直せば従来の方式に戻すことがでるというわけです。  
![3DtouchConnect1](https://github.com/Toshi2020/KINGROON-KP3_3Dtouch_WiFi_UI-mod/assets/81674805/ce54e505-5ddc-45e6-a523-7cc569fa1680)
![3DtouchConnect2](https://github.com/Toshi2020/KINGROON-KP3_3Dtouch_WiFi_UI-mod/assets/81674805/f9404c74-8e5b-4ac5-b9f6-cc5157ad1571)

**●注意すべき点**  
・出荷時期によるのかと思いますが私のKP3はステッピングモーターの配線が逆になっているのでそれ用のConfiguration.hの設定となっています。よってフラッシュした後いきなりホーミングをせずに、XYを少し動かしてみて方向を確認し、逆に動くようであればConfiguration.hの3412行から8行分をコメントアウトしてください。Configファイルの変更点はファイルの末尾にまとめてあります。最初に表示されるステータス画面中ほどのXYZの表示をタップするとマニュアルでヘッドを移動できる画面になります。(ちなみにその下の緑のアイコンと100%の部分をタップするとフィードレートとフローレートを変更できます。これって変更するものなんですかね？)  
・3Dtouchの3ピンプラグの信号はマザーボード側とピンの並びが違うので、絶対に直結しないように！3Dtouchかマザーボードのどちらかが焼損すると思われます。通電前にしっかり確認してください。2ピン側も然りです。  
・3Dtouchのセンサ出力が正しく認識されているかを確認するために、最初のZ軸のホーミングは少し上の位置から始めて、ヘッドがベッドに達する前にセンサのピンを指で戻してモーターが停止することを確認してください。正しく接続できていないとベッドに穴が開くことになります。  
・もともと使っていたZ軸の原点検出のマイクロスイッチは使わないので、本体側のねじを締めて、Z軸が目いっぱい下がってもマイクロスイッチがぶつからないようにしておくと良いと思われます。  
・これも出荷時期によると思いますが、KP3にはヘッド部分の穴位置が異なるものがあるようで、私の手元の物はセンターが左に4mm、奥に6mmずれているのを修正しています。これもお手持ちのKP3に合わせてConfiguration.hを修正してください。一辺160mmの正方形のラインのモデルを作成してスカートなしで印刷してみると良いかと思います。  
  
・MKS WiFiモジュールにはTFT35用とrobin mini用の2種類があって、搭載しているチップは同じですが生えている足が異なります。今回はMKS robin WiFiと呼ばれている足が2列の物を調達する必要があります。  
  
・変更したファイルの動作については無保証です。特にWiFi関連は十分な検証ができていません。WiFiを組み込まない場合はConfiguration.hの#define ROBIN_WIFIの行をコメントアウトたほうがいいかもしれません。WiFiモジュールを挿さずにWiFiありのビルドをフラッシュしてもSSIDが"TP-LINK_MKS"、IPアドレスが"192.168.3.100"と表示されるだけかとは思いますが。  
  
・Marlinは初回を除きプログラムをフラッシュしてもEEPROMの内容は変化しません(フラッシュの度に初期化する設定にもできますが今回はデフォルトのままです)。よってDEFAULT_で始まるパラメータを変更してビルドしてフラッシュしたら、Configuration/Advanced Settings/initialize EEPROMを実行するか、まずConfiguration/Restore Defaultsを実行し、続いて/Store Settingsを実行しないとEEPROMに反映されないと思います。  

**●使い方**  
・3Dtouchのキャリブレーションのやり方ですが、Motion/Z Probe Wizardでウイザードが起動します。これを実行するとZ軸のホーミングの後でノズルが中央に移動するので、ノズルとベッドの隙間を紙一枚(0.15mm)程度に調整してDoneで抜けます。その後Configuration/Store Settingsを忘れずに。ノズルを交換しない限り一度やればOKかと思っています。値をメモしておけばEEPROMをイニシャライズした場合にConfiguration/Probe Z Offsetで値を直接設定できます。  
・自動レベリングに関しては、Motion/Bed Levelingで行った後Configuration/Store Settingsでセーブする必要があります。ベッドの種類を交換するたびに行う必要があり頻度がそれなりに高いと思われるので、メニューのCustom Commandsの下にこの2つを続けて行うコマンドを用意しました。(私はPLA専用とPETG&TPU用の磁気マットを使い分けています)  
・私は柔らかな磁気マットを主に使っているためうねりが大きい可能性があるので、5x5の25点を計測する設定としています。ガラスしか使わないなら3x3点でもいいのかも。  
・なおレベリングしたデータはホーミングを行うと自動で反映される設定にしてあるので、スライサー側のコマンドを追加しなくても大丈夫です。  
・ベッドの高さ調整の4か所の調整ネジはバネで押さえていますがこれが勝手に動いてしまうのが頻繁に高さ調整しなくてはいけなかった原因だと思います。私はバネを取り外し、代わりにカラーとしてM4のナットを2個づつ入れてゆるまないように固定しました。そのため印刷の度にレベリングを行う必要はないと思っています。  
  
・LCDに表示される印刷経過時間は、オリジナルのMarlinではヒーターオンのタイミングでカウントを開始するのですが、これでは実際の印刷時間とはいえないと思いますし、予想終了時間がめちゃくちゃになります。そのため今回Marlin側では自動でカウントを開始しない設定とし、スライサー側の初期コマンドのG28(ホーミング)の後で、M75(プリントタイマー開始)を追加しています。(ただし、Marlin側の操作で連続して印刷を行うと、2回目の印刷ではベッド温度が下がらない限りタイマーは自動スタートしてしまうようです。)元に戻すにはConfiguration.hの3436行の#undef PRINTJOB_TIMER_AUTOSTARTをコメントアウトします。  
・ノズルとベッドのPID自動調節メニューはConfiguration/Advanced Settings/Temperature/PID Autotuneです。実行後Configuration/Store Settingsでセーブする必要があります。私はPCとKP3をシリアル接続してM503コマンドで読みだした値をConfiguration.hに記述しておきました。  
・フィラメントのロード/アンロードメニューはChange Filamentの下にあります。アンロードはボーデン型のエクストルーダーに合わせてデフォルトで330mmとしています。  
  
・WiFiのSSIDとキーの設定は、"WiFi_NameKey.txt"というテキストファイルにSSIDとキーの2行だけを記述して、SDカードに入れて電源を入れると設定される仕様にしました。TFT35では設定するメニューがあるのですが、そこまで凝る必要はないと思います。WiFiモジュール側に記憶されるので一度行えばOKです。使い終わったファイルの拡張子は他に合わせて.cur(おそらくカレントの意味)に変更するようにしました。  
・起動後、接続する時間を少し待ってからAbout Printer/Printer Infoを選んで画面をスクロールすると一番下にSSIDとIPアドレスが表示されます。IPは後で必要なので覚えておきます。  
・SSIDの上の行はMKS WiFi自体のファームウエアバージョンです。MKSによればこれがCS1.0.4_200227より古い場合はファームウエアのアップデートを行う必要があるとのことです。先のMKSのリンク先からfirmware_releaseディレクトリ下のzipをダウンロードして解凍したMksWifi.binをSDカードにコピーしてKP3を起動するとアップデートが始まります。約1分で終了します。  
・SSIDとKeyおよびファームアップデートは、PCのブラウザから先のIPアドレスにアクセスするとwebページが表示されるので、そこからも変更が可能です。(初回はファイルを使って設定する必要があるのではと思いますが。)  
  
・CuraからWiFi接続をするためにはプラグインの導入が必要です。MKSのGitHubにはダウンロードした物を手動で組み込めという説明がありますが、私の環境ではCuraの起動時に読み込みエラーが出て使えませんでした。が、次のやり方でいけることがわかりました。  
・Curaを起動して右上のマーケットプレースのボタンを押して、検索でmksと入れるとMKS WiFi Plugin 1.4.2が出てきますので、そちらをインストールします。その後Curaの再起動が必要だったかと思います。Curaのバージョンは5以上が必要とどこかに書いてあったと思います。  
・Curaプリンタの管理画面にWiFi設定が追加されるので、KP3のIPアドレスを追加してWiFiを使うのチェックボックス2か所にチェックを入れ、Advanced settingsでStart printing after file uploadにチェックを入れておけばファイル転送後に自動で印刷がスタートします。  
・Curaのモニタータブで温度等を見たりプレヒートができ、MoreボタンでPC上のGコードファイルを転送したりファンのオンオフやヒーターやモーターのオフなどができます。プリント時間の表示が変ですがこれはCura側の問題だと思います。  
・今回WiFiからKP3のSDカードへ転送するファイル名は"#WiFi#.gcode"に固定しています。これはMarlinが日本語を含むファイル名を扱えない(KINGROONのKP3ファームは日本語のフォントが組み込まれていたこともあり扱えた)のと、CuraがWiFi経由だと同名ファイルの上書きができないことが気に入らなかったためです。このためCuraからSDカード内のファイル一覧は見えますが、そこから"#WiFi#.gcode"以外のファイルを指定して印刷したりデリートしたりはできません(もちろんKP3本体の操作でのファイル選択や、PCにある任意のGコードのファイルを転送することはできます)。今回のやり方はSDカードのファイルはテンポラリなバッファとして扱うということですね。3Dではない普通のプリンターに印刷するようなイメージです。Configuration.hの編集で#define FIXED_FILENAMEの行をコメントアウトすれば本来の仕様になります。  
・WiFiモジュールとMarlinの間はシリアル通信を行っています。ビットレートは通常は115200bpsでファイル転送時には1958400bpsとなり、この時Marlin側は割り込み方式からDMA方式に切り替えて受けるようになってます。非同期シリアル通信としては高速ですが、USBで直接SDに書き込むよりも何百倍も遅いので、例えば3DBenchyの場合は転送に44秒かかりました。転送後のファイルサイズが3,966,727バイトなので転送速度は約90kバイト/秒です。  
・MKSの説明では100kバイト/秒となっているので、まあこんなものかと。Curaの見積もりで3DBenchyの推定印刷時間は1時間44分なので、転送時間はトータルの0.7%に過ぎない、とも言えます。SDカードの抜き差しから解放されるメリットの方が大きいと感じています(私のKP3はSDカードスロットのロック機構が壊れたためバネを殺しているので、カードの挿入位置がシビアという事もあります)。  
  
**★2023 11/13 追記**  
・MKSのWiFiモジュール用ファームのソースを眺めていたら、ファイル転送時のビットレート1958400bpsを設定する行の前後に、ビットレートを4500000bpsとする行がコメントアウトで残されていることがわかりました。DMAなのだからこの程度の速度まで行けるのかも？と思いトライしてみました。  
・WiFiモジュールのファームのソースは上記MKSのGitHubの物です。  
・gcode.cppとgcode.hのエンコードをUTF-8に変更してセーブしなおしました(中華文字が入っているので)。  
・MksWifi.inoの1684と1689行あたりの4500000と1958400の行のコメントアウトを入れ替えました(2か所)  
・ビルドはArduinoIDE 2.2.1を利用し、GitHubの説明の通りにビルドしMksWifi.binを作成しました。  
・Marlin側ではwifiSerial.hのWIFI_UPLOAD_BAUDRATEを1958400から4500000に変更してビルドしました。  
・以上の変更により、3DBenchyの転送時間は44秒から29秒に短縮されました。  
・MKSとしては想定外の設定なのかもしれませんが、しばらくはこれで使ってみようと思っています。  

**●変更したファイルの概要**  
・私の修正した箇所は///の3本線でコメントアウトしています。"KP3"の文字も入れたので検索しやすいと思います。(入れ忘れてる個所もあるかもですが)  
・日本語のコメントが入っているファイルのエンコードはUTF-8になってます。  
  
・platformio.ini  
→KINGROON KP3のマザーボードはrobin miniなのでdefault_envsを変更しています。  
  
・Marlin\Configuration.h  
・Marlin\Configuration_adv.h  
→各種設定の変更。ベースとなったのは先のMarlinのサイトのConfigurationのView / DownloadからConfigurations/config/examples/Kingroon/KP3/の下のConfiguration.hとConfiguration_adv.hです。  
→変更はベース部分には手を付けず、ファイルの最後にまとめて追記しています。これは変更箇所を後からいちいち探すのが面倒だったからです。そのためベース部分で#defineで定義済みの項目はいったん#undefで未定義とし、パラメータを書き換える必要がある時には改めて#defineしています。変更理由などはファイルにコメントとして記述してあります。  
・なおメニューはデフォルトのまま1回目のタッチでハイライトの移動、再度タッチで選択を行う設定になってます。1回のタッチで選択を行う設定にもできるのですが、その場合本当に正しい場所をタッチしたのかどうかわからない、という事があるためです。  
  
・Marlin\src\gcode\stats\M31.cpp  
→オリジナルでは印刷終了時にステータス行に経過時間が表示されるのですが、これを行わないように変更しました。経過時間はプログレスバーの上に既に表示されているので、ステータス行に同じ情報を、しかも別フォーマットで出すのはお間抜けなので。  
  
・Marlin\src\lcd\menu\menu_configuration.cpp  
→オリジナルではメニューのEEPROMのセーブ、ロード、デフォルト値の読み込みを、選択するとただちに実行してしまいますが、確認画面を挿入するように変更しました。画面が小さいので間違って隣を選択してしまうことがあり、その場合に実行をキャンセルできるようにするためです。  
  
・Marlin\src\lcd\menu\menu_info.cpp  
→プリンタのステータス画面にWiFiモジュールのファームバージョン、SSID、IPアドレスの表示を追加しました。IPアドレスはCuraの設定に必要です。  
  
・Marlin\src\lcd\tft\tft_image.h  
→メニューの上下スクロールボタンの画像が左右を向いているのを上下向きに変更しました。感覚と合わないので。  
  
・Marlin\src\lcd\tft\touch.cpp  
→画面をタッチした時にクリック音を出すように変更しました。画面が小さいのでタッチしたことをフィードバックした方が良いと感じたので。Configuration.hの#define TOUCH_SOUNDの行をコメントにすれば消すことはできます。  
  
・Marlin\src\lcd\tft\touch.h  
→画面をタッチし続けた時のリピート間隔を大きくしました。デフォルトではリピートが早すぎて、Zプローブウイザードでノズルが一気に下がって磁気マットに穴をあけてしまったので。  
  
・Marlin\src\lcd\tft\ui_320x240.cpp  
→メニュー画面下部のスクロールボタンの表示位置を画面の両端に寄せ、各ボタンの周囲に枠を付けてタッチ可能エリアを判別できるようにしました。メニューの一番下の項目を選択するときにボタンをタッチしてしまうことが度々あったので。  
→ファンの回転時に表示色が変わるようにしました。ホットエンドとベッドは加熱時に色が変わるのですが、パーツ冷却ファンだけ色が変わらないのは統一性がないと感じたので。  
→プログレスバーのすぐ上には印刷経過時間が表示されるのですが、追加でプログレスの%表示と、せっかくなので推定終了時間を表示するようにしました。%表示はKINGROONのファームで表示されていてそれなりに役立っていたので。推定終了時間はMarlinが計算しているものをもらっていますが、まあ参考程度です。  
  
・Marlin\src\lcd\tft\ui_320x240.h  
→メニュー画面の1行の高さを1dot減らすことで、メニューの一番下の行の下端とボタンの上端との間隔を広げました。ボタンを操作したつもりでメニューが選択されてしまうことがあったので。  
  
・Marlin\src\libs\buzzer.h  
→メニューからEEPROMの操作を完了したときなどに、エラーを彷彿とさせる「ピイィィ」というビープ音が鳴って、何？と思うので、OKの場合の音を「ピピ」に変更しました。  
  
・Marlin\src\pins\stm32f1\pins_MKS_ROBIN_MINI.h  
→WiFiモジュールの制御ピンの定義が間違っているのでコメントアウトしました。(Configuration_adv.hで再定義しています。)  
  
・Marlin\src\RobinWiFi_KP3フォルダ以下  
→WiFiモジュールとMarlinの間を取り持つソフトウェア群です。これはMarlin\src\lcd\extui\mks_uiフォルダのTFT35用のWiFi関連のソースをコピーして、UI関連を削除してrobin mini用に改修した物です。  
  
・Marlin\src\MarlinCore.cpp  
→WiFiモジュールのイニシャライズとWiFiポーリング処理を追加しました。  
