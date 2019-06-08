# Lab 1: AWS DeepRacer の強化学習モデルを構築しましょう!

# 注意
AWS DeepRacer サービスはより良い顧客体験を提供するため継続的に改善を重ねています。以前のコンテンツは古くなっている可能性があるため、GitHub の最新の Lab コンテンツを参照してください。もし技術的な質問がある場合はワークショップのファシリテーターに、自宅で Lab に取り組む方は [AWS DeepRacer forum](https://forums.aws.amazon.com/forum.jspa?forumID=318) (英語) に質問をポストしてください。


# 概要
この Lab には4つのゴールがあります: 

1. AWS console 内の AWS DeepRacer の使い方に慣れること、
2. モデルをトレーニングする際に必要な概念を説明すること、
3. 仮想サーキット・Summit サーキットでの AWS DeepRacer リーグの競い方を説明すること、そして、
4. モデルを向上させる方法を説明することです。

Lab は3つのセクションに分割されています: 

1. Section 1: はじめのモデルをトレーニングする、
2. Section 2: AWS DeepRacer リーグで競う、
3. Section 3: モデルのトレーニングとモデルの向上。

ゴール1と2は Section 1、ゴール3は Section 2、ゴール4は Section 3 でカバーされます。


# 形式
Lab を完了するのに50分かかります。全てのコンテンツを試し、サービス概観し、はじめの強化学習 (reinforcement learning; RL) モデルをトレーニングするのに十分な時間です。Section 1 は約25-30分、Section 2 は約5分、Section 3 はワークショップの時間よりかかると思いますが、家で試せるようになっています。Lab では AWS マネージメントコンソールで AWS DeepRacer の様々な構成要素についての詳細を紹介するので、全てを試してみることができます。Section 1 の最後でモデルのトレーニングを始めます。


# ヒント
- 仮アカウントを使っている場合、報酬関数を保存し、トレーニング済みのモデルをダウンロードするようにして下さい。Summit 終了後はアカウントへのアクセスができなくなり、アカウントは削除されます。
- 早くトレーニングジョブを始めたい人も、時間をとって予めコンセプトをよく理解し、それからモデルのトレーニングを始めることをお勧めします。
- Lab の進捗に従って質問して下さい。テーブルでの議論は自由に行ってください。
- 最後にトレーニングジョブを始める際、re:Invent トラックでは少なくとも90分はジョブを走らせて下さい。必要なサービスが立ち上がるのに6分かかり、モデルが完走するまでにレーストラックを探索する時間が必要です。
- もし Lab が終わっても学習を継続したい場合は、AWS トレーニングおよび認定チームによって提供される [AWS DeepRacer: Driven by Reinforcement Learning](https://www.aws.training/learningobject/wbc?id=32143) をご確認下さい。

# Section 1: はじめのモデルをトレーニングする
## Step 1: AWS DeepRacer サービスにログインする
提供されたアカウント情報を用いて [AWS Console](https://signin.aws.amazon.com/signin?redirect_uri=https%3A%2F%2Fconsole.aws.amazon.com%2Fconsole%2Fhome%3Fnc2%3Dh_ct%26src%3Dheader-signin%26state%3DhashArgs%2523%26isauthcode%3Dtrue&client_id=arn%3Aaws%3Aiam%3A%3A015428540659%3Auser%2Fhomepage&forceMobileApp=0) にログインして下さい。

**米国東部（バージニア北部）/ North Virginia** リージョンが選択されていることを確認し、[AWS DeepRacer](https://console.aws.amazon.com/deepracer/home?region=us-east-1) (https://console.aws.amazon.com/deepracer/home?region=us-east-1) を開いて下さい。

AWS DeepRacer のランディングページから、左ペーンを開き、**Reinforcement learning** を選択します。

## Step 2: モデルリストのページ
Reinforcement learning を選ぶと、モデルページにたどり着きます。このページはあなたが作成したすべてのモデルのリストと、それぞれのモデルの状態を表示します。もしモデルを作成したい場合、このページから始めます。同様に、このページから、モデルのダウンロード・複製・削除が行えます。

![Model List Page](img/model_list_deepracer.png)

モデルがない場合、リストは空なので、**Create model** を選択してモデルを作成することができます。

モデルを作成するとこのページからステータス (例えば Training, Ready) を見ることができます。モデルステータスが Ready であるとは、モデルのトレーニングが完了し、ダウンロード・評価・仮想レースへの提出ができることを示します。モデル名をクリックし、**Model details** ページに進むことができます。


## Step 3: モデルの作成
このページで AWS DeepRacer のための RL モデルを作成し、モデルのトレーニングを開始します。ページ内にいくつかのセクションがありますが、それぞれを見る前に、まずページの一番下までスクロールして全体を眺めて、上まで戻ってきて下さい。これから AWS DeepRacer で使われる、レーストラックを自動運転で走る (アクションをとる) モデルを作成しようとしています。特定のレーストラックを選び、モデルが選択できるアクションを定義し、期待される運転方法を決めるための報酬関数を定義し、トレーニングに用いられるハイパーパラメータを設定します。


### <font color=blue>**Info**</font> **ボタン**
コンソールでの操作を通して <font color=blue>**Info**</font> ボタンが目に入ると思います。これを選択すると右からスクリーンに情報ペーンが現れます。Info ボタンは、Information ペーンのリンクを選択しない限り他のページには遷移しません。完了したらペーンを閉じることができます。


## 3.1 Model details

Model details 画面の初めから設定していきます。ここでは、モデルの名前と説明を設定することができます。もしこのサービスを使用するのが初めての場合、**Create Resources** ボタンをクリックします。この操作によって DeepRacer が他の AWS サービスを使用するために必要な IAM ロールが作成されます。他の AWS サービスとは、学習と評価で使用するVPCスタック、Python 3 の報酬関数を検証するための AWS DeepRacer lambda 関数、モデルアーティファクトを保存するためのAWS DeepRacer S3 バケットなどです。この手順でエラーが出た場合はお知らせください。

![Model Details](img/model_details.png)

モデルの名前と説明を入力し、次のセクションまでスクロールしてください。


## 3.2 Environment simulation
ワークショップで詳しく説明されているように、RL モデルのトレーニングはシミュレータの仮想レーストラックで行われます。このセクションでは、モデルをトレーニングするトラックを選択します。 AWS RoboMaker を使用してシミュレーション環境を立ち上げます。

モデルのトレーニングを行うときは、参加したいレースのトラックを想定し、参加する予定の最終トラックに最も似ているトラックでトレーニングしてください。これは必須ではありませんし、また、良いモデルを保証するものでもありませんが、レースで最高のパフォーマンスを発揮する可能性を最大限に高めます。さらに、直線のトラックでトレーニングする場合は、曲がり方を学ばせることは期待できません。

AWS DeepRacer リーグの詳細はセクション2で説明します。このセクションでは、リーグのレースに参加する場合に、トレーニングするレースの選択にあたって注意すべきことを説明します。

- [Summit Circuit](https://aws.amazon.com/deepracer/summit-circuit/) では、ライブレーストラックは re:Invent 2018 トラックです。そのため、AWS Summit のレースに参加したい場合は re:Invent トラックでモデルをトレーニングしてください。
- 仮想サーキットのそれぞれのレースには独自の競技トラックがありますが、競技トラックで直接トレーニングすることはできません。その代わりに、全く同じではありませんが、テーマやデザインが似たトラックをご利用いただくことが可能です。これによりモデルが一般化され、競技トラックに過剰適合することがなくなります。

本日のラボでは Summit でのレースの準備をしていただきたいと思います。re:Invent 2018 トラックを選択して Action space の設定画面までスクロールしてください。


## 3.3 Action space

このセクションでは、トレーニング中や、トレーニング済みのモデルから選択したアクションスペースを設定します。アクションとは、速度とステアリング角の組み合わせです。AWS DeepRacer では、連続アクションスペースではなく離散アクションスペースを使用します。離散アクションスペースをビルドするには、Maximum speed、Speed granularity、Maximum steering angle、Steering granularity を設定する必要があります。


![action space](img/Action_Space.png)

パラメーターの説明

- Maximum steering angle は、車が曲がる際のフロントホイールの左右それぞれの最大角度です。このパラメーターはどれくらいホイールを回転させるかの上限を決めるもので、最大値は30度です。
- Steering angle granularity は、左右それぞれの最大ステアリング角の分割数です。Maximum steering angle を30度に設定した場合、左側に+30度、右側に-30度傾けることができます。Maximum steering angle を30度に設定し、Steering angle granularity に5を設定すると、ステアリング角は左から右に向かって、30度，15度、0度、-15度、-30度の5分割になります。ステアリング角は0度を中心として左右対称です。
- Maximum speed は、車がシミュレータ内を走行する最高速度です。
- Speed granularity は、0 m/sから、設定した最高速度の範囲の分割数です。アクションスペースでの速度の範囲は、0 m/sは含まれず、設定した最高速度を含みます。Speed granularity に3、Maximum speed に3を設定した場合、アクションスペースでの速度は 1 m/s、2 m/s、3 m/s が使用可能です。これは単純に 3 m/s ÷ 3 ＝ 1 m/s の計算をし、0 m/s から 3 m/s の間を 1 m/s ずつ増加させていることを意味します。


上記の例の場合、最終的なアクションスペースには15個の個別のアクション（速度が3種類 x ステアリング角が5種類）が含まれます。この情報は AWS DeepRacer サービスに表示されます。もしまだ設定されていない場合、アクションスペースを設定してください。お好きな設定をしていただいて構いません。なお、アクションスペースが大きくなるとトレーニングに少し時間がかかるかもしれません。

ヒント

- モデルはアクションスペースに無いアクションをしません。同様に、たとえば、直線トラックでモデルをトレーニングした場合、カーブを曲がることができません。つまりロバストなモデルを作りたい場合は、アクションスペースとトレーニング用トラックに注意する必要があります。
- 速い速度や広いステアリング角を設定するのは良いことですが、報酬関数やフルスピードでカーブに突っ込んだり直線でジグザグ走行しないかについて考慮する必要があります。
- 我々の実験では、最高速度が速いモデルは、遅いモデルよりも収束に時間がかかりました。
- 実際のレースにあたっては、AWS DeepRacerのWebサーバーユーザーインターフェースで実行して、シミュレータよりも車が速く走らないことを確認する必要があります。


## 3.4 報酬関数
強化学習において、報酬関数はモデルをトレーニングする上で**非常に重要な**役割をもっています。報酬関数は、トレーニングモデルが行動決定を行う際にとってほしい行動に報酬を与えるために使われます。

報酬関数は、ある行動から得られる結果を評価し、その行動に報酬を与えます。実際には、報酬はトレーニング中、各行動が取られた後に計算され、経験という形（ステート、アクション、次のステート、報酬）でモデルのトレーニングに使われます。報酬関数のロジックはシミュレータによって提供される変数によって構築することができます。これらの変数は車からの測定値を表しており、例えば、ステアリング角度、スピード、レーストラック上での (x, y) 座標や経路情報となります。これらの測定値を使い、独自の報酬関数ロジックを Python 3 シンタックスを利用して実装することができます。

次のテーブルは、報酬関数で利用できる変数を表しています。これらの変数は、エンジニアやサイエンティストがよりよい方法を見つけるたびにアップデートされるため、適時作った報酬関数を調整するよう注意してください。Singapore Summit (2019年4月10日) 時点での変数名・説明は AWS コンソールと同じです。AWS DeepRacer サービス内の最新の情報を常に使うようにしてください。SageMaker RL ノートブックを利用される場合は、ノートブック自体のシンタックスを確認するよう注意してください。


| 変数名        | シンタックス                                                                                | 型                     | 説明                                                                                                                                                                                                                                                                                                                                                         |
|----------------------|---------------------------------------------------------------------------------------|--------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| all_wheels_on_track  | params['all_wheels_on_track']                                                         | Boolean                  | 4輪全てがトラック（走行路または線）上にある場合、`all_wheels_on_track` は `True` となります。1輪でもトラックの外にある場合、`all_wheels_on_track` は `False` となります。                                                                             |
| x                    | params['x']                                                                           | Float                    | 車の前車軸の中心の `x` 座標をメートル単位で返します。                                                                                                                                                                                                                                                                                |
| y                    | params['y']                                                                           | Float                    | 車の前車軸の中心の `y` 座標をメートル単位で返します。                                                                                                                                                                                                                                                                                 |
| distance_from_center | params['distance_from_center']                                                        | Float [0, track_width/2] | トラックの中心からの絶対距離。トラックの中心は全ての `waypoints` の中心が繋げられた線により定義されます。                                                                                                                                                                                                                                      |
| is_left_of_center    | params['is_left_of_center']                                                           | Boolean                  | 車がトラックの中心から左側に位置するかどうかを示します。                                                                                                                                                                                                                                                                                     |
| is_reversed          | params['is_reversed']                                                                 | Boolean                  | 車がトラックの順方向でトレーニングしているのか、逆方向でトレーニングしているのか。                                                                                                                                                                                                                                    |
| heading              | params['heading']                                                                     | Float (-180,180]           | 車の先頭の向いている角度を示します。`x` 軸が増加する方向（`y` 軸は固定）に向いている場合、0を返します。`y` 軸が増加する方向（`x` 軸は固定）の場合、`90` を返します。`y` 軸が減少する方向（`x` 軸は固定）の場合、`-90` を返します。 |
| progress             | params['progress']                                                                    | Float [0,100]            | 完了したトラックの割合をパーセンテージで示します。`100` はラップの完了を示します。                                                                                                                                                                                                                                                                                   |
| steps                | params['steps']                                                                       | Integer [0,inf]                 | 完了したステップを返します。 1ステップは1つの `(state, action, next state, reward)` タプルに対応します。                                                                                                                                                                                                                                                                               |
| speed                | params['speed']                                                                       | Float                    | 期待する車のスピードがメートル/秒で返されます。これは選択されたアクションに結び付けられます。                                                                                                                                                                                                                                                               |
| steering_angle       | params['steering_angle']                                                              | Float                    | 度単位で表される、ステアリングの角度。定義した Action Space に紐づきます。注: 正の値は左向きを表し、負の値は右向きを表します。これは2次元平面上で処理されます。                                                                                       |
| track_width          | params['track_width']                                                                 | Float                    | トラックの幅を表します                                                                                                                                                                                                                                                                                                                             |
| waypoints            | params['waypoints'] for the full list or params['waypoints'][i] for the i-th waypoint | List                     | トラックの中心の順序付きリストで、各要素は `(x, y)` 座標です。リストのインデックスは `0` から始まります。                                                                                                                                                                            |
| closest_waypoints    | params['closest_waypoints'][0] or params['closest_waypoints'][1]                      | Integer                  | 車の現在地から最も近い `waypoint` のインデックス。`params['closest_waypoints'][0]` は後方のインデックスを、`params['closest_waypoints'][1]` は前方のインデックスを示します。                                                                                                          |
|                      |                                                                                       |                          |                                                                                                                                                                                                                                                                                                                                                                     |
|                      |                                                                                       |                          |                                                                                                                                                                                                                                                                                                                                                                     |
|                      |                                                                                       |                          |                                                                                                                                                                                                                                                                                                                                                                     |


以下は報酬関数に利用できるいくつかのパラメータを図示したものです。

![rewardparams](img/reward_function_parameters_illustration.png)

以下の折れ線グラフは re:Invent トラック上の、waypoints を可視化した画像です。報酬関数の中では、waypoints の中心線のみアクセスすることができます。報酬関数内で waypoints のリストを出力しプロットすることで、このグラフを自作することができます。print 関数を報酬関数内で利用する場合、出力は AWS RoboMaker logs に置かれます。どのトラックにおいても行うことができます。詳細は後の方に記します。

![waypoints](img/reinventtrack_waypoints.png)

報酬関数を考える上で役立つ方法は、車を上手に走らせるにはあなたならどうするのかを考えることです。簡単な例は、路上に留まり続ける車に対して報酬を与えるものです。これは reward = 1 とすることで行うことができます。DeepRacer のシミュレーター上では、車がトラックの外に出た場合にはリセットし、トラックを再スタートするため、トラックを外れる行動に対して報酬を考える必要はありません。しかし、この報酬関数は、関数内部で利用できる他の全ての変数を全く利用していないため、おそらく最も優れた報酬関数とは言えません。

以下にいくつかの報酬関数の例を示します。

**Example 1**: 中心線に沿った動きに報酬を与える基本的な報酬関数
ここでは、3つのマーカーによって、3つの走行帯をトラックの周りに作ります。そして、狭い走行帯を走る車により高い報酬を与えます。報酬の大きさには違いがあり、狭い走行帯で走るものには、1を、真ん中の走行帯で走るものには0.5を、一番広い走行帯を走るものには0.1を与えます。もし、狭い走行帯の報酬を減らすか、真ん中の走行帯の報酬を増やすと、トラックの走行面を広く使うように報酬を与える形になります。これは、特に鋭いカーブに有効です。


	def reward_function(params):
		'''
		Example of rewarding the agent to follow center line
		'''
		
		# Calculate 3 marks that are farther and father away from the center line
		marker_1 = 0.1 * params['track_width']
		marker_2 = 0.25 * params['track_width']
		marker_3 = 0.5 * params['track_width']
		
		# Give higher reward if the car is closer to center line and vice versa
		if params['distance_from_center'] <= marker_1:
			reward = 1.0
		elif params['distance_from_center'] <= marker_2:
			reward = 0.5
		elif params['distance_from_center'] <= marker_3:
			reward = 0.1
		else:
			reward = 1e-3  # likely crashed/ close to off track
		
		return float(reward)

ヒント: 0の値を報酬としてを与えないでください。AWS DeepRacer内で使用されているオプティマイザーは報酬に0を与えられると混乱します。0の値を与える代わりに、報酬を小さな値で初期化します。

**Example 2**: 極端なステアリングにペナルティーを与え、中心線に沿った動きに報酬を与える高度な報酬関数


	def reward_function(params):
		'''
		Example that penalizes steering, which helps mitigate zig-zag behaviors
		'''
	
		# Calculate 3 marks that are farther and father away from the center line
		marker_1 = 0.1 * params['track_width']
		marker_2 = 0.25 * params['track_width']
		marker_3 = 0.5 * params['track_width']
	
		# Give higher reward if the car is closer to center line and vice versa
		if params['distance_from_center'] <= marker_1:
			reward = 1
		elif params['distance_from_center'] <= marker_2:
			reward = 0.5
		elif params['distance_from_center'] <= marker_3:
			reward = 0.1
		else:
			reward = 1e-3  # likely crashed/ close to off track
	
		# Steering penality threshold, change the number based on your action space setting
		ABS_STEERING_THRESHOLD = 15
	
		# Penalize reward if the car is steering too much
		if abs(params['steering_angle']) > ABS_STEERING_THRESHOLD:  # Only need the absolute steering angle
			reward *= 0.5
	
		return float(reward)

**Example 3**: 速度を落とすのにペナルティーを与え、中心線に沿った動きに報酬を与える高度な報酬関数


	def reward_function(params):
		'''
		Example that penalizes slow driving. This create a non-linear reward function so it may take longer to learn.
		'''
	
		# Calculate 3 marks that are farther and father away from the center line
		marker_1 = 0.1 * params['track_width']
		marker_2 = 0.25 * params['track_width']
		marker_3 = 0.5 * params['track_width']
	
		# Give higher reward if the car is closer to center line and vice versa
		if params['distance_from_center'] <= marker_1:
			reward = 1
		elif params['distance_from_center'] <= marker_2:
			reward = 0.5
		elif params['distance_from_center'] <= marker_3:
			reward = 0.1
		else:
			reward = 1e-3  # likely crashed/ close to off track
	
		# penalize reward for the car taking slow actions
		# speed is in m/s
		# the below assumes your action space has a maximum speed of 5 m/s and speed granularity of 3
		# we penalize any speed less than 2m/s
		SPEED_THRESHOLD = 2
		if params['speed'] < SPEED_THRESHOLD:
			reward *= 0.5
	
		return float(reward)

これらのサンプルを利用し、自身の報酬関数を作りましょう。以下に他のTipsを示します。

- waypoints を利用し、ある waypoint と次の waypoint から方向を計算することができます。
- 2D ゲームにおける右手の法則を利用することで、トラックのどっち側にいるか判断することができます。
- 指数関数的に報酬を与えることができますが、最大10,000に制限してください。
- 報酬関数で speed と steering_angle を利用する際は、Action Space を覚えておいてください。
- ログ内で、ラップを完走した episode を追跡するには、progress = 100 の際に、完走ボーナス (つまりは reward += 10000) を与えることを考慮にいれてください。これは、一度車がラップを完了した後は、progress は100以上にはなりませんが、シミュレーションは続くためです。モデルは終了時間に到達するまでトレーニングを続けますが、特に実際のレースでは、最終モデルがベストなモデルとは言えないためです。これは一時的なワークアラウンドになります。

報酬関数を作り終えたら、**Validate** ボタンを押し、コードシンタックスが正しいことをモデルのトレーニング前に確認してください。トレーニングを開始すると、報酬関数はファイルとして S3 に保存されまずが、念のためコピーし、他の場所へ保存してください。

これが、1つめの例を利用した報酬関数の例になります。

![rewardfunction](img/NewReward.png)

次のセクションにスクロールしてください。

## 3.5 アルゴリズム設定
このセクションでは、強化学習で使われるハイパーパラメータを指定します。ハイパーパラメータを指定することでパフォーマンスが改善します。

実施する前に、用語について説明します。

経験としても知られる **step** は、 (s,a,r,s’) のタプル型です。s はカメラによってキャプチャされた観測（または状態）、a は車両によって行われた行動、r は行動によって生じる報酬、s’ は行動から得られた新しい観測（または新しい状態）です。

**episode** は、車両がスタート地点から出発し、最終的にトラックを完走するか、またはトラックから外れるまでの期間です。つまり、 step (つまり経験) の連続が episode になります。異なる episode は、長さが異なる場合があります。

**experience buffer** は、トレーニング中、一定数の、順序付けられた step からなる様々な長さのepisodeで構成されています。experience buffer は、ニューラルネットワークを更新するための基本情報（方策と価値）として機能します。

**batch** は、順序付けられた経験のリストで、一定期間に渡るシミュレーションで得られた経験の一部を表し、policy network の重みの更新で使用されます。

experience buffer からランダムにサンプリングされた **batch** のセットを **training data set** と呼び、policy network の重みを更新するのに使用されます。

これらのパラメータは専門家によってテストおり、re:Invent のトラックと、小さな action space でうまく機能するようになっているので、あまり変更しないでください。ただし、トレーニングの収束性を大幅に改善することができるので、モデルのイテレーションを開始するときに変更することを検討してください。


| Parameter                                | Description                                                  |
| ---------------------------------------- | ------------------------------------------------------------ |
| Batch size                               | 車両の経験は、experience buffer からランダムでサンプリングされ、ディープニューラルネットワークの重みを更新するため使用されます。例えば、experience buffer に5120の経験があり、ランダムサンプリングを無視してバッチサイズを512を指定すると、10のバッチ得られます。各バッチは、重みの更新に使用されます。バッチサイズを大きくした場合、重みの更新は安定しますが、トレーニングが遅くなる可能性があるので注意してください。 |
| Number of epochs                         | エポックは、すべてのバッチを1回実行することを表し、重みは各バッチが処理される毎に更新されます。例えば、10 エポックの場合、すべてのバッチを使用して重みを更新するプロセスを10回繰り返すことを意味します。エポックを増やすことでより安定して重みを更新できますが、トレーニングが遅くなります。バッチサイズが小さい場合は、少ない epoch を使用できます。 |
| Learning rate                            | 学習率は、重みを更新する大きさを制御します。簡単に言えば、方策の重みを変えて累積報酬を最大にしたい場合、方策をどれだけ変えるべきかということです。学習率を大きくするとトレーニングは早くなりますが、収束しにくくなります。学習率を小さくすると収束は安定しますが、トレーニングに時間がかかります。 |
| Exploration                              | 探索と搾取の間のトレードオフを決めるために使用する方法を指定します。言い換えると、いつ探索（ランダムに行動）をやめ、これまでの経験を利用するべきかを決定するためにどのような方法を使うべきか、ということです。個別の action space を使うので、必ず CategoricalParameters を選択してください。 |
| Entropy                                  | action space の確率分布に追加された、ある程度の不確実性、ランダム性。ランダムにアクションを選択して 状態/action space をより広く探索するのに役立ちます。 |
| Discount factor                          | 割引率は、将来の報酬が予想累積報酬にどれだけ寄与するかを指定します。割引率を大きくすると、モデルは予想累積報酬を決定するためにより長い将来を見るため、トレーニングが遅くなります。例えば割引率が0.9の場合、車両は将来の ~10 ステップに含まれる報酬によって移動を決定します。0.999の割引率であれば、将来の ~1000 ステップに含まれる報酬を考慮して移動します。推奨される割引率は、0.99、0.999、および0.9999です。 |
| Loss type                                | 損失タイプは、重みを更新するために使用される目的関数（コスト関数）のことです。Huber と平均二乗誤差は、小規模な更新の場合同様に動作します。しかし、更新が大きくなるに連れて Huber の損失は、平均二乗誤差の損失に比べて増分が小さくなります。学習の収束に問題が有る場合、Huber を使います。収束性がよく、より早くトレーニングしたい場合は平均二乗誤差を使用してください。 |
| Number of episodes between each training | このパラメータは、各モデルのトレーニング・イテレーション毎に車両がどれだけ経験を得るべきかを制御します。例えば多くの極大値を持つ複雑な問題の場合、experience buffer は多くの無相関データが必要です。大きくした場合、トレーニングは遅くなりますが安定します。推奨値は、10、20、40です。 |

各トレーニング・イテレーション後に新しいモデルは S3 バケットに保存します。AWS DeepRacer サービスは、トレーニング実行中にすべてのモデル表示するのではなく、最後のモデルだけを表示します。これらについては Section 3 を参照してください。


## 3.6 停止条件
トレーニングを始める前の最後のセクションです。ここではモデルがトレーニングする最大時間を指定できます。可能であれば、この条件を入力すべきです。いつでも早くトレーニングをやめることが出来ます。もし入力した条件の結果としてモデルが停止した場合は、モデル一覧画面に遷移し、モデルを複製して新しいパラメータを使用してトレーニングを再開できます。

90分を指定してから、**Start training** を選択してください。エラーになった場合、エラー箇所に移動します。Python のシンタックスも検証されます。トレーニングを開始したら、トレーニングを開始するために必要なサービスが起動するまでに最大6分かかります。この間に AWS DeepRacer リーグと、どのように参加するのかを説明します。


![Stopping conditions](img/stop_conditions.png)

この時点までに25〜35分の実験時間が経過しているはずです。

ヒント: 報酬機能の保存を確認し、トレーニング済みモデルをバーナーアカウントからダウンロードしてください。Summit 後にアカウントへのアクセスを失い、削除されます。


# Section 2: AWS DeepRacer リーグで競う
[AWS DeepRacer リーグ](https://aws.amazon.com/deepracer/league/)は世界初のグローバル自走型レーシングリーグです。このリーグは、2019年に複数都市での直接およびオンライン上で開催されます。レースをし、賞または47人に与えられる re:invent 2019 で行われる AWS DeepRacer Knockout Rounds への参加権および旅費を勝ち取りましょう。その予選を勝ち抜くと、AWS DeepRacer Champioonship Cup に参加できます。詳細は[こちら](https://d1.awsstatic.com/DeepRacer/AWS-DeepRacer-League-2019-Official-Rules-English-29-April-2019(1).pdf)を参照してください。

直接参加型レースはSummit サーキット、オンラインレースは仮想サーキットを参照してください。Summit サーキットイベントの場所は[こちら](https://aws.amazon.com/deepracer/summit-circuit/)から探してください。仮想サーキットの詳細は AWS DeepRacer サービスが一般向け公開されてから発表されます。どの競技に参加する場合でも、AWS DeepRacer を所有する必要はありません。


![League](img/league.png)


## Summit サーキットでレースをする
Summit サーキットでレースするには、トレーニング済み AWS DeepRacer RL モデルを USB メモリースティックの models フォルダに入れ、Summitに持参する必要があります。また、モデルを持参できなかった方にも、レースを体験していただくことのできる標準モデルを提供する予定です。イベントでは、先着順またはレーススタッフの進行に応じて、用意したモデルを使い AWS DeepRacer カーをトラック上でレースさせ、4分間での最速ラップタイムを計測します。AWS Summit のレースに合わせて準備する際、レーストラックは re:Invent 2018 トラックを予定しているので、このトラックに合わせてモデルをトレーニングしましょう。各 Summit サーキットで最速の参加者は re:Invent への参加権を、そしてトップ10には、AWS DeepRacer カーを勝ち取れます。

## 仮想サーキットでレースする
仮想サーキットでレースするには、AWS コンソール内の AWS DeepRacer サービスよりモデルを送信し、それぞれのレースにエントリーしてください。仮想サーキットのレースは AWS DeepRacer サービスの [DeepRacer Virtual Circuit](https://console.aws.amazon.com/deepracer/home?region=us-east-1#leaderboards) にあります。

![VirtualCircuit](img/dvc.png)

スクロールすると、開催中のレースを見つけられます。

![VirtualCircuitOpen](img/dvc-ll.png)

レースの詳細は Race Information から見ることができます。

![VirtualCircuitInfo](img/dvc-info.png)

トレーニング済みモデルを作成すると、現在の開催中のレースにそのモデルを送信できます。モデルは、指定されている競技トラックを使って AWS DeepRacer サービスによって評価されます。評価後、ラップタイムが前回の結果よりよければ、アップデートされた新しいタイムを確認することができます。

![VirtualCircuitModelSubmit](img/model-submitted.png)

仮想サーキットでは、各レースごとに新しい競技トラックが使われるため、直接その競技トラックに向けてトレーニングを行うのは難しいです。全く同じというわけではありませんが、競技トラックのテーマとデザインに似たトラックが用意されます。つまり、モデルは一般化される必要があり、競技トラックに過学習されるべきではありません。仮想サーキットの各レースで最速の参加者はre:Inventへの招待券、そしてトップ10には AWS DeepRacer カーを勝ち取るチャンスがあります。

**Tip**: DeepRacer サービスは現在、モデルのインポートをサポートしておりませんが、model.tar.gz ファイルと、全てのトレーニングの成果物を保存できます。最終モデルは model.tar.gz として、DeepRacer の S3 バケットの DeepRacer-SageMaker-rlmdl-account number-date というフォルダに保存されます。中間モデルは、DeepRacer-SageMaker-RoboMaker-comm-account number-date フォルダに .pd ファイルとして保存されます。

Summit サーキットと仮想サーキットの各イベントの後に、全ての参加者は完走時間を元にポイントを受け取ります。ポイントはシーズン毎に集計され、最高ポイント獲得者は re:Invent に招待されます。詳細は [AWS DeepRacer リーグの利用規約](https://aws.amazon.com/deepracer/faqs/#AWS_DeepRacer_League) を参照してください。 

# Section 3: モデルのトレーニングと改善

## 3.1: モデルのトレーニング

モデルのトレーニングが開始したら、モデルの一覧からモデルを選択できるようになります。モデルの一覧では、エピソードごとの報酬の合計値のグラフからトレーニングの進行状況を確認することができます。また、シミュレータ上の一人称視点の画像を見ることができます。

初めのうち、車はまっすぐ走らないかもしれませんが、車がより良いふるまいを学習するにつれてパフォーマンスが向上し、報酬グラフが上昇していくことが確認できるでしょう。なお、車がトラックを外れた場合はトラックがリセットされます。その際、車が同じ場所から走行を開始しなくても問題ありません。これは、ラウンドロビンを有効にして前回コースアウトした場所から次の走行を始めるようにすることで、トラック全体をトレーニングするためです。さらに、トレーニング中に車がトラックを反対方向に走ることがありますが、これは左右のカーブをバランス良くトレーニングしてモデルをより汎化させるためです。最後に、車がコースアウトしたにも関わらずトラックがリセットされないことがありますが、この現象はトレーニングによって得られた情報を Amazon SageMaker に送ってモデルを学習させるタイミングで発生します。更新されたモデルが AWS RoboMaker に送信されると走行は再開します。

ログファイルは、Amazon SageMaker と AWS RoboMaker のどちらからも確認することができます。ログは Amazon CloudWatch に出力されます。ログを見るには、報酬グラフの右上にある更新ボタンの下にある3つのドットをクリックして **View logs** を選択します。

![Training Started](img/widg.png)

ここでは Lambda、Amazon SageMaker、AWS RoboMaker のログを見ることができます。

![Logs](img/view_in_logs.png)

各フォルダには、AWS DeepRacer で実行したすべてのトレーニングジョブのログが格納されています。AWS RoboMaker のログにはシミュレータの出力が記録され、Amazon SageMaker のログにはモデルの学習ログが記録されます。エラーが発生した場合は、これらのログを確認すると良いでしょう。

![AWS RoboMaker Logs](img/robomaker_logs.png)

AWS DeepRacer では、Amazon SageMaker、AWS RoboMaker、Amazon S3、Amazon Kinesis Video Streams、AWS Lambda、Amazon CloudWatch を使用しています。これらのサービスを使用することによって、各サービスのステータスの確認やその他の有用な情報を得ることができます。

これらのサービスでは、保存されている現在や過去のジョブのリストを見ることができます。こちらの画像は、Amazon SageMaker で実行された学習ジョブの一覧です。

![SageMaker jobs](img/sagemaker_listjobs.png)

Amazon SageMaker では、学習を実行するために使用したEC2インスタンスの使用状況のログを見ることができます。

![SageMaker jobs](img/sagemaker_jobdetails.png)

AWS RoboMaker では、シミュレーションジョブの一覧を見ることができます。アクティブなジョブに関しては、シミュレーション環境を直接見ることができます。

![RoboMaker jobs](img/aws_robomaker_jobs_list.png)

アクティブなシミュレーションジョブを一覧から選択し、Gazebo アイコンをクリックします。

![RoboMaker job details](img/aws_robomaker.png)

新しいウィンドウが開き、シミュレーション環境が表示されます。**ここで加えた変更は即座にシミュレーションに反映されてしまうのでご注意ください。うっかり車や環境をドラッグしたり回転したりすると、トレーニングジョブに良くない影響を与えることがあります。**

![RoboMaker simulator](img/robomaker_simulator.png) 

Amazon Kinesis Video Stream は，通常、使用終了後のスペース解放やストリームの数の制限によって削除されます。現在、トレーニングと評価のどちらにおいても、動画はS3に格納されません。

![KVS stream](img/kvs_stream_video.png)

Amazon S3 は AWS DeepRacer で参照する最終モデルと、トレーニングジョブ中にトレーニングされた中間モデルを aws-deepracer バケットに格納します。報酬関数も同じバケットに保存されます。

![S3list](img/s3.png)

最終モデルは model.tar.gz ファイルとして DeepRacer S3 バケットの DeepRacer-SageMaker-rlmdl-アカウント番号-日時、という名前のフォルダに保存されます。
中間モデルは拡張子が pd のファイルとして DeepRacer-SageMaker-RoboMaker-comm-アカウント番号-日時、という名前のフォルダに保存されます。

![S3dr](img/s3_aws_deepracer.png)

AWS DeepRacer サービスは、現時点では各トレーニングジョブにおいてひとつの最終モデルのみ参照することができます。ただし、トレーニングの途中で得られたモデルを使用したい場合は、model.pd ファイルを最後の model.tar.gz ファイルと入れ替えることができます。モデルが壊れる可能性があるため、tar.gz ファイルの中のファイルは変更しないでください。この操作は、モデルのトレーニングが終了してから、もしくはトレーニングを手動で停止してから行ってください。

## 3.2: モデルの性能評価

ワークショップでは、ステップ2以降を行う時間が無いかもしれません。モデルのトレーニングが完了したらモデルの評価を行うことができます。モデルの詳細画面で **Start evaluation** をクリックすると、モデルの性能と周回数の評価を行いたいトラックを選択することができます。re:Invent 2018 トラックと5周を選択してStartをクリックしてください。

評価が完了すると、以下のように表示されます。

![evaluation_done](img/evaluation_done.png)

## 3.3: AWS DeepRacer リーグでのレース

もしモデルに満足したら、[Summit Circuit](https://aws.amazon.com/deepracer/summit-circuit/) のレースか、今なら [Virtual Circuit](https://console.aws.amazon.com/deepracer/home?region=us-east-1#leaderboards) に参加できます。トレーニング済みのモデルを [ここ](https://console.aws.amazon.com/deepracer/home?region=us-east-1#leaderboards) から仮想サーキットで現在開催中のレースに提出して下さい。

## 3.4: モデルの向上と試行錯誤

モデルの評価に基づき、トラックを期待通り完走できたかや、平均ラップタイムについて知ることができます。なお、仮想サーキットのレースでは一定の周回数を連続して走りきる必要があるため、信頼性の高いモデルを構築する必要があります。周回数はレース毎に決定されます。

現時点では、報酬関数とハイパーパラメータを試行錯誤する必要があります。異なる運転特性に基づいた少数の異なる報酬関数を試し、シミュレータで評価し、一番よいものを選ぶといいでしょう。もし AWS DeepRacer をお持ちであれば、実機でテストすることも可能です。

ヒント: 
- トレーニング時間を長くする。もしモデルが期待通り完走できなければ、トレーニング時間を伸ばしてみて下さい。
- 最大速度を上げて行動空間を修正し、速いラップタイムを達成する。
- 報酬関数を微調整し、車がより速く走ることに対してインセンティブを与える。すなわち変数 (progress, steps, speed) を修正する。
- 過去のトレーニングを活かせるようモデルを複製する。ただし、モデルを複製した場合、行動空間を変更することはできません (変更するとジョブが失敗します)。

## 3.5: RoboMaker のログを見てモデルのパフォーマンスを分析する

さらに一歩進めたい場合、ログファイルを調べることでトレーニングジョブ実行間の各モデルのパフォーマンスを評価することができます。

CloudWatch からログファイルをダウンロードするには、[AWS CLI](https://docs.aws.amazon.com/polly/latest/dg/setup-aws-cli.html) を使って以下のように行います。

**CloudWatch から RoboMaker ログのダウンロード**


1. [Quick Analysis] ログから最新の 10000 を取得

  ```aws logs get-log-events --log-group-name  "/aws/robomaker/SimulationJobs"  --log-stream-name  "<STREAM_NAME>" --output text --region us-east-1 > deepracer-sim.log```

2. [Export Entire Log] Amazon Cloudwatch から Amazon S3 へログをコピー。次のリンクに全てのログを出力方法が記載されています: [Amazon S3](https://docs.aws.amazon.com/AmazonCloudWatch/latest/logs/S3ExportTasks.html)

Python Pandas を用いてログファイルを分析し、どのモデルが最も高い報酬を得ているかがわかります。さらに、完走ボーナスを加えると、どのモデルがラップを完了できたか知ることができます。これらのモデルはシミュレータでテストし、実世界で走らせるためのいい候補となるでしょう。


