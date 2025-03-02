# DigitalCurling3-ClientExamples

このリポジトリはデジタルカーリング思考エンジンクライアントの作成例を提供します．

- 思考エンジンのテンプレートについてはリポジトリ [DigitalCurling3-SimpleClient](https://github.com/digitalcurling/DigitalCurling3-SimpleClient) を参照ください．
- 思考エンジンの開発方法については[思考エンジンの開発方法](https://github.com/digitalcurling/DigitalCurling3/wiki/%E6%80%9D%E8%80%83%E3%82%A8%E3%83%B3%E3%82%B8%E3%83%B3%E3%81%AE%E9%96%8B%E7%99%BA%E6%96%B9%E6%B3%95)を参照ください．

## ビルド

1. このリポジトリをクローンします．
1. サブモジュールをセットアップするために， `git submodule update --init --recursive` を実行します．
1. [Boost](https://www.boost.org/)をインストールします．
1. [CMake](https://cmake.org/)をインストールします．
1. CMakeがユーザーの `PATH` に入っていることを確認します．
1. 下記のビルドコマンドを実行します．

```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . --config Release
```

:warning: CMakeがBoostを見つけられない場合は，環境変数`BOOST_ROOT`にBoostをインストールしたディレクトリを設定してください．

## 思考エンジンクライアント作成例

### stdio

ショットの初速と回転を標準入力から入力できるようにしたものです．

### rulebased

簡単なif-thenルールに従い，1ターン先のシミュレーションを行う思考エンジンです．
シミュレータFCV1について初速を逆算する関数 `EstimateShotVelocityFCV1()` が含まれます．

アルゴリズム：

1. No.1ストーンが敵チームのものならばNo.1ストーンをテイクアウトする．
   No.1ストーンを目標地点とした反時計回り，時計回りのショットについて，それぞれ同数シミュレーションを行い，テイクアウトできた回数が多いほうを採用する．
2. No.1ストーンが自チームのものならば，その2m手前にガードストーンを置く．
3. No.1ストーンがハウス内にないなら，ティーの位置にストーンを置く．

## ライセンス

The Unlicense