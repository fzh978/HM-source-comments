### 介绍

  该项目对HEVC_Test_Model(HM)视频编码(encoder)部分的所有源码进行了详细的注释和理解，涵盖了整个完整的视频编码过程，源码版本为HM16.16．解码(decoder)部分与编码部分共用大量的函数库，且较编码部分更为简单，故未注解．

  HEVC_Software源码可使用SVN从官方服务器下载，[https://hevc.hhi.fraunhofer.de/svn/svn_HEVCSoftware/(https://hevc.hhi.fraunhofer.de/svn/svn_HEVCSoftware/).

### 推荐参考书籍
1.  ***High Efficiency Video Coding Algorithms and Architectures***. 该书由多位HEVC标准的参与者和带头人合力著写而成，如MIT的Vivienne Sze，Texas Instruments的Madhukar Budagavi，Microsoft的Gary J. Sullivan等，其权威性毋庸置疑，也是HEVC/H.265标准最最重要的参考书籍．作者的表达能力和文字功底也十分了得，在兼具行文逻辑性的同时也将复杂的原理解释的十分清晰准确,阅读起来流畅舒适．

2. ***新一代高效视频编码H.265/HEVC原理、标准与实现***.　该书对原理部分的介绍较少，更加侧重工程实现部分，因此是阅读源码时不错的参考，但某些地方容易让人产生困惑，可以结合上一部推荐书籍一起阅读．

3. ***Recommendation ITU-T H.265*** 该书更像是一本手册，侧重解释视频编解码中的语法语义的设计、含义和求得过程．在阅读源码的过程中对参数求值有不理解的地方也许与语法语义的设计有关．

### 源码阅读建议

 a. 阅读源码前需对HEVC编码原理有足够清晰详细的了解，推荐书籍1和2就是不错的资料． 

 b. 建议采用从下至上的方式阅读源码，选取自己对原理十分熟悉的一小部分底层代码开始阅读．如帧内预测部分，这样对代码的变量命名方式、变量的含义和设计方式等慢慢了解，再逐渐向高层的代码阅读，这样不至于从顶层一开始就迷失在无尽的变量和各种不清楚功能的类中．

 c. 尽管推荐书籍1和2,尤其是书籍1对编码原理做了清晰的介绍，但远远不够详细，不可能包含代码中的所有细节，这时需要多加思考，有时还需要参考具体的JCTVC提案和论文．

 d. 该软件为了最大限度的提高运行速度，常见的数学计算基本使用位运算实现，如乘法、除法、取余、掩码等(基本不直接使用除法及很少使用乘法)，还有许多方法本身就是基于二进制位运算实现的，如二进制算术编码，因此需要对位运算足够的熟悉．
　如果不可避免的需要多次重复的使用浮点数除法，如需要计算某些理论推导得到的数学公式，这种情况下一般对可能的输入值带入公式事先计算好结果，将计算结果(左)移位取整后制成rom表放入程序中直接使用，牺牲小部分内存空间而大大加快程序执行速度．
  HEVC是基于四叉树的块结构实现整个编码过程的，在率失真优化过程中会多次涉及四叉树的深度遍历，来求得最优的块分割方式，因此也需要对深度遍历(递归)足够熟悉．

 e. 对于部分逻辑较为复杂或不好理解的代码片段可扣取出来，举出具体实例，分析中间结果，理解程序是如何工作的．












