# Changelog for package rgbd_sensor

hhp_1.0.6RC1 (2022-09-09)
------------------
1. 新增CP3AM型号摄像头的标定文件
2. 将默认的相机标定文件读取路径更换为绝对路径
3. 增加launch文件中的参数：相机标定文件读取路径

hhp_1.0.6 (2022-08-30)
------------------
1. RGBD sensor完成x3派适配，可用转接板接到x3派上使用
2. 新增读取相机标定文件并发布相机内参的功能

v1.0.2 (2022-07-20)
------------------
1. 合入最新RGBD厂家SDK
2. 使用opencv接口优化NV12转RGB数据耗时，提升整体输出帧率

v1.0.1 (2022-06-23)
------------------
1. 解决 rgbd 通过shared_mem 传输存在的问题