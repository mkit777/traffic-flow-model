# traffic-flow-model

## 01 简介

交通流动力学模型仿真实现

本仓库主要目的是为了复现论文[高速跟驰交通流动力学模型研究](https://kns.cnki.net/kcms/detail/detail.aspx?dbcode=CJFD&dbname=CJFDLAST2020&filename=WLXB202006015&v=2o%25mmd2BPGdiSUM31yswLKG6c%25mmd2B47g8ygAGI%25mmd2BUu25PDvJIDv8jaS5fLSBej%25mmd2BJT7lvVZVYB)

实现了三个模型：

* NaSch
* STCA
* HCCA

相关介绍见如下文章

* [【交通流动力学模型01】单车道 NaSch 模型介绍及实现](https://mp.weixin.qq.com/s/qLxEUlN0QGIMW3cAU-Bd2w)

* [【交通流动力学模型02】双车道 STCA 模型介绍及实现](https://mp.weixin.qq.com/s/AgHuJ3bmG4cM7DfBdOwD6A)

* [【交通流动力学模型03】高速跟驰 HCCA 模型介绍及实现](https://mp.weixin.qq.com/s/7C3W7yWWcR_AUAFuIerLqg)

## 02 使用方式

1. 创建并切换虚拟环境

   ```bash
   python3 -m venv venv
   
   # 激活虚拟环境,windows
   .\venv\Scripts\activate
   # 激活虚拟环境,mac/linux
   source ./venv/bin/activate 
   ```

2. 安装依赖

   ```
   pip install matplotlib
   ```

3. 运行各个文件即可

