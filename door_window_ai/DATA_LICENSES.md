# 数据许可与来源记录

本精简运行时包不包含训练图片、标注、数据集下载清单或训练脚本。以下内容是历史训练来源记录，不能替代各数据集当前的许可证文本。

## Open Images V7（历史来源）

- 官方下载页：[Open Images V7](https://storage.googleapis.com/openimages/web/download_v7.html)
- 历史流程使用过 validation image/bbox 子集，并保留 `ImageID`、原始 URL、作者和许可字段。
- 任何重新下载、训练、公开样本或再分发，都必须以原始页面和当前数据许可为准，并满足署名要求。

## DeepDoors2（未随包提供）

官方项目：[gasparramoa/DeepDoors2](https://github.com/gasparramoa/DeepDoors2)。本次发布包未包含该数据集。若后续使用，必须单独核对其下载条件、`boundary_semantics` 和再分发限制。

## 伪掩码

历史流程中的伪掩码是由 Open Images bbox 派生的标注，不是人工 GT 或官方 segmentation annotation；伪掩码本身不会产生新的数据许可。

## 未使用的数据集

ZInD、Structured3D、ADE20K Full、Hypersim 未随本包下载或分发。训练数据、训练日志和报告均不属于本次软件发布范围。

本文档采用 CC BY-NC-SA 4.0。各数据集及原始图片仍遵循其各自许可证和署名要求。
