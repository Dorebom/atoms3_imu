このプログラム自体はMIT LICENCEです。

M5Stack AtomS3 Liteを使って、IMUの姿勢推定データをバイナリでUART通信するプログラムです。

(現状は、テキストデータで送信してる)

M5Stack AtomS3 Liteのビルド環境は、SAITO Tetsuyaさんのコードを利用。
https://github.com/3110/m5stack-platformio-boilerplate-code
MIT LICENCE

UARTのパケットシリアライズは、Christopher Bakerさんのコードを利用。
https://github.com/bakercp/PacketSerial
MIT LICENCE

姿勢推定は、arduino-librariesを利用。
https://github.com/arduino-libraries/MadgwickAHRS
LGPL LICENCE

(ライセンス関連の記載もあとあと早い段階でちゃんとしたい)

事前準備

- VS codeをインストール
- VS codeを起動して、拡張機能からPlatformIO IDEをインストール

プロジェクトを開く

- VS codeの左側のバーからPlatformIO IDE(アリの顔マーク)を選択
- (左側のフォルダ・ファイル表示部が切り替わるので)QUICK ACCESS>>PIO Home>>Openを選択
- Open Projectを押して、このREADME.mdがあるフォルダ(platformio.iniが選択画面で見えてる状態)を選択
- (初回は、関連するライブラリを読み込んで、ビルド環境を自動構築するため結構待つ)完了

ビルド方法

- VS codeの左側のバーからPlatformIO IDE(アリの顔マーク)を選択
- (プロジェクトを一度でもオープン済みなら見れる)PROJECT TASKSから`m5stack-atoms3-lite-m5unified`のフォルダを開く
- General直下のBuildを選択

M5Stack AtomS3 Liteへの書き込みは、同様に、General直下のUploadを選択