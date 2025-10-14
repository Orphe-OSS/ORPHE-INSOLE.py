# ORPHE-CORE.py
Happy hacking for ORPHE INSOLE with python!!


> [!CAUTION]
> 現在ORPHE INSOLEでは運動解析処理（Analytics）は利用できません。Analyticsを利用したい場合はORPHE COREをご利用ください。
  

## Requirements
 * Python 3.10 or later

## Installation
```bash
git clone https://github.com/Orphe-OSS/ORPHE-INSOLE.py.git
cd ORPHE-INSOLE.py
pip install bleak
```

## 動作確認

### 周りのBLEデバイスをスキャンする
特定のinsoleデバイスに接続したい場合は、orphe.connect()の引数にデバイスのアドレスを指定することができます。これを利用するにあたって、特定のコアモジュールのアドレスを知りたい場合は以下を実行して周りのBLEデバイスをすべてスキャンすることができます。
```bash
python scan.py
```

### sensor valuesの値を取得する
加速度センサ、ジャイロセンサ、クオータニオンセンサ、圧力センサの値を取得することができます。
以下を実行して、sensor valuesの値を取得することができます。 詳細はソースコードを参照して下さい。
終了する場合は`Ctrl+C`で終了できます。
```bash
python get_sensor_values.py
```

### device informationを取得する
device_information.py にコアモジュールの情報を取得するサンプルコードがあります。以下を実行して、コアモジュールの情報を取得することができます。
```bash
python device_information.py
```

### matplotlibを利用してデータを可視化する

`plot_acc_values.py`または`plot_pressure_values.py`を実行することで、取得したセンサデータをmatplotlibを利用してリアルタイムに可視化することができます。初期設定では加速度値や圧力センサデータを可視化していますが、他のデータを可視化したい場合は適宜変更してください。
```bash
pip install matplotlib
python plot_acc_values.py
# または
python plot_pressure_values.py
```

### OSCでデータを送信する
`osc.py`を実行することで、ORPHE COREから取得したデータをOSCで送信することができます。初期設定では5005番のポートに送信します。なお`osc.py`は加速度値のみをoscで送信していますので、他のデータを送信したい場合は適宜変更してください。
```bash
pip install python-osc
python osc.py
```

具体的なアプリケーション側での受信方法については [OSCで様々なアプリケーションにデータをリアルタイム送信する](https://github.com/Orphe-OSS/ORPHE-CORE.py/wiki/%E7%95%AA%E5%A4%96%E7%B7%A8.-OSC%E3%81%A7%E6%A7%98%E3%80%85%E3%81%AA%E3%82%A2%E3%83%97%E3%83%AA%E3%82%B1%E3%83%BC%E3%82%B7%E3%83%A7%E3%83%B3%E3%81%AB%E3%83%87%E3%83%BC%E3%82%BF%E3%82%92%E3%83%AA%E3%82%A2%E3%83%AB%E3%82%BF%E3%82%A4%E3%83%A0%E9%80%81%E4%BF%A1%E3%81%99%E3%82%8B)を参照してください。この解説では ORPHE COREモジュールを対象としているので適時読み替えて下さい。

### GUIを利用してINSOLEと接続する
<img align="left" width="200" height="auto" src="gui.png">

`gui.py`にORPHE INSOLEの接続や切断をgui上から実行できます。非常にシンプルなアプリケーションですが、INSOLEの電波状況による接続解除や手動接続解除、バッテリーレベル表示等を実装しています。guiには tkinter というpythonの標準ライブラリを利用していますので、追加のインストールは不要です。フロントユーザレベルアプリケーション開発の参考にしてください。またOSCと組み合わせることで簡単なOSC送信アプリケーションも作成できます。

```bash
python gui.py
```

## ドキュメント
  * [ORPHE CORE Python API Reference](https://orphe-oss.github.io/ORPHE-INSOLE.py/api/orphe_insole.html)

### 生成方法
orphe_insole.pyのdocstringからドキュメントを生成します。htmlファイルの生成には pdoc3 を利用しています。orphe_insole.pyのdocstringを書き換えたり、機能を追加した場合は以下のコマンドでドキュメントを再生成してください。
```bash
pip install pdoc3
pdoc orphe_insole --html -o docs/api --force
```

## 教材
wikiに学習教材や具体的なケーススタディをまとめています。このREADMEで基本的な使い方を学んだ後は、wikiを参照してさらに深く学ぶことができます。
  * [ORPHE-CORE.py学習教材](https://github.com/Orphe-OSS/ORPHE-CORE.py/wiki)
  * ORPHE-INSOLE.py学習教材 （準備中）

 
