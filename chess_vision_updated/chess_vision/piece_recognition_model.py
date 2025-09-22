#!/usr/bin/env python3

import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.transforms as transforms
from torchvision.models import resnet18
import cv2
import numpy as np
import os
from typing import Tuple, List, Optional
import logging

class ChessPieceCNN(nn.Module):
    """象棋棋子识别的卷积神经网络模型"""

    def __init__(self, num_classes=15, input_size=64):
        """
        初始化CNN模型

        Args:
            num_classes: 分类数量 (0=空, 1-7=红方, 8-14=黑方)
            input_size: 输入图像大小
        """
        super(ChessPieceCNN, self).__init__()

        self.input_size = input_size
        self.num_classes = num_classes

        # 使用预训练的ResNet18作为backbone，但替换最后的分类层
        self.backbone = resnet18(pretrained=True)
        # 修改第一层以接受单通道输入（灰度图）
        self.backbone.conv1 = nn.Conv2d(1, 64, kernel_size=7, stride=2, padding=3, bias=False)

        # 替换分类层
        self.backbone.fc = nn.Sequential(
            nn.Dropout(0.5),
            nn.Linear(512, 256),
            nn.ReLU(inplace=True),
            nn.Dropout(0.3),
            nn.Linear(256, num_classes)
        )

        # 初始化新层的权重
        self._init_weights()

    def _init_weights(self):
        """初始化权重"""
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
            elif isinstance(m, nn.BatchNorm2d):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.Linear):
                nn.init.normal_(m.weight, 0, 0.01)
                nn.init.constant_(m.bias, 0)

    def forward(self, x):
        """前向传播"""
        return self.backbone(x)


class PieceRecognitionEngine:
    """棋子识别引擎"""

    def __init__(self, model_path: Optional[str] = None, device: str = 'cuda'):
        """
        初始化识别引擎

        Args:
            model_path: 预训练模型路径
            device: 计算设备 ('cuda' 或 'cpu')
        """
        self.device = torch.device(device if torch.cuda.is_available() else 'cpu')
        self.model = ChessPieceCNN()
        self.model.to(self.device)

        # 数据预处理管道
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((64, 64)),
            transforms.Grayscale(),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.5], std=[0.5])
        ])

        # 类别标签映射
        self.class_names = {
            0: 'empty',      # 空格
            1: 'red_king',   # 红帅
            2: 'red_rook',   # 红车
            3: 'red_horse',  # 红马
            4: 'red_cannon', # 红炮
            5: 'red_bishop', # 红相
            6: 'red_advisor',# 红士
            7: 'red_pawn',   # 红兵
            8: 'black_king', # 黑将
            9: 'black_rook', # 黑车
            10: 'black_horse',# 黑马
            11: 'black_cannon',# 黑炮
            12: 'black_bishop',# 黑象
            13: 'black_advisor',# 黑士
            14: 'black_pawn'  # 黑卒
        }

        # 加载预训练模型
        if model_path and os.path.exists(model_path):
            self.load_model(model_path)
            self.model.eval()
        else:
            logging.warning(f"Model path {model_path} not found. Using untrained model.")

        self.model.eval()

    def load_model(self, model_path: str):
        """加载预训练模型"""
        try:
            checkpoint = torch.load(model_path, map_location=self.device)
            if 'model_state_dict' in checkpoint:
                self.model.load_state_dict(checkpoint['model_state_dict'])
            else:
                self.model.load_state_dict(checkpoint)
            logging.info(f"Model loaded from {model_path}")
        except Exception as e:
            logging.error(f"Failed to load model from {model_path}: {e}")

    def save_model(self, save_path: str, epoch: int = 0, loss: float = 0.0):
        """保存模型"""
        try:
            torch.save({
                'epoch': epoch,
                'model_state_dict': self.model.state_dict(),
                'loss': loss,
                'class_names': self.class_names
            }, save_path)
            logging.info(f"Model saved to {save_path}")
        except Exception as e:
            logging.error(f"Failed to save model to {save_path}: {e}")

    def extract_square_image(self, image: np.ndarray, square_position: Tuple[int, int],
                           square_size: int = 50) -> np.ndarray:
        """
        从棋盘图像中提取单个格子的图像

        Args:
            image: 原始图像
            square_position: 格子位置 (x, y)
            square_size: 格子大小

        Returns:
            格子图像
        """
        x, y = square_position
        h, w = image.shape[:2]

        # 确保坐标在图像范围内
        x1 = max(0, x - square_size // 2)
        y1 = max(0, y - square_size // 2)
        x2 = min(w, x + square_size // 2)
        y2 = min(h, y + square_size // 2)

        square_img = image[y1:y2, x1:x2]

        # 如果提取的区域太小，进行填充
        if square_img.shape[0] < square_size or square_img.shape[1] < square_size:
            square_img = cv2.resize(square_img, (square_size, square_size))

        return square_img

    def predict_piece(self, square_image: np.ndarray) -> Tuple[int, float]:
        """
        预测单个格子的棋子类型

        Args:
            square_image: 格子图像

        Returns:
            (piece_type, confidence) 棋子类型和置信度
        """
        try:
            # 预处理图像
            if len(square_image.shape) == 3:
                square_image = cv2.cvtColor(square_image, cv2.COLOR_BGR2RGB)

            # 应用变换
            input_tensor = self.transform(square_image).unsqueeze(0).to(self.device)

            # 推理
            with torch.no_grad():
                outputs = self.model(input_tensor)
                probabilities = F.softmax(outputs, dim=1)
                confidence, predicted = torch.max(probabilities, 1)

                piece_type = predicted.item()
                confidence_score = confidence.item()

            return piece_type, confidence_score

        except Exception as e:
            logging.error(f"Error in piece prediction: {e}")
            return 0, 0.0  # 默认返回空格

    def recognize_board(self, board_image: np.ndarray,
                       square_positions: List[Tuple[int, int]]) -> Tuple[List[int], List[float]]:
        """
        识别整个棋盘的棋子布局

        Args:
            board_image: 棋盘图像
            square_positions: 90个格子的位置坐标列表

        Returns:
            (piece_types, confidences) 棋子类型列表和置信度列表
        """
        piece_types = []
        confidences = []

        for position in square_positions:
            # 提取格子图像
            square_img = self.extract_square_image(board_image, position)

            # 预测棋子
            piece_type, confidence = self.predict_piece(square_img)

            piece_types.append(piece_type)
            confidences.append(confidence)

        return piece_types, confidences

    def detect_board_changes(self, current_board: List[int],
                           previous_board: List[int]) -> List[int]:
        """
        检测棋盘变化的位置

        Args:
            current_board: 当前棋盘状态
            previous_board: 之前棋盘状态

        Returns:
            变化位置的索引列表
        """
        changes = []
        for i, (curr, prev) in enumerate(zip(current_board, previous_board)):
            if curr != prev:
                changes.append(i)
        return changes

    def validate_chess_position(self, board_state: List[int]) -> Tuple[bool, str]:
        """
        验证象棋局面是否合法

        Args:
            board_state: 棋盘状态

        Returns:
            (is_valid, error_message) 是否合法和错误信息
        """
        # 统计棋子数量
        piece_counts = {}
        for piece in board_state:
            piece_counts[piece] = piece_counts.get(piece, 0) + 1

        # 检查帅/将数量
        if piece_counts.get(1, 0) != 1:  # 红帅
            return False, "红帅数量不正确"
        if piece_counts.get(8, 0) != 1:  # 黑将
            return False, "黑将数量不正确"

        # 检查其他棋子数量限制（简化版本）
        max_counts = {2: 2, 3: 2, 4: 2, 5: 2, 6: 2, 7: 5,  # 红方
                     9: 2, 10: 2, 11: 2, 12: 2, 13: 2, 14: 5}  # 黑方

        for piece_type, max_count in max_counts.items():
            if piece_counts.get(piece_type, 0) > max_count:
                piece_name = self.class_names.get(piece_type, f"piece_{piece_type}")
                return False, f"{piece_name}数量超过限制"

        return True, ""

    def get_piece_name(self, piece_type: int) -> str:
        """获取棋子名称"""
        return self.class_names.get(piece_type, f"unknown_{piece_type}")


class DataGenerator:
    """训练数据生成器"""

    def __init__(self, data_dir: str):
        """
        初始化数据生成器

        Args:
            data_dir: 数据目录路径
        """
        self.data_dir = data_dir
        self.create_directories()

    def create_directories(self):
        """创建数据目录结构"""
        dirs = ['train', 'val', 'test']
        for split in dirs:
            for class_idx in range(15):  # 0-14类别
                dir_path = os.path.join(self.data_dir, split, str(class_idx))
                os.makedirs(dir_path, exist_ok=True)

    def generate_synthetic_data(self, num_samples_per_class: int = 100):
        """
        生成合成训练数据

        Args:
            num_samples_per_class: 每个类别的样本数
        """
        logging.info("开始生成合成训练数据...")

        # 这里应该实现实际的数据生成逻辑
        # 可以使用Gazebo仿真环境生成不同角度、光照条件下的棋子图像
        # 或者使用数据增强技术从少量真实图像生成更多样本

        for class_idx in range(15):
            for sample_idx in range(num_samples_per_class):
                # 生成合成图像（示例）
                synthetic_img = self._generate_piece_image(class_idx)

                # 保存到训练集
                save_path = os.path.join(self.data_dir, 'train', str(class_idx),
                                       f'synthetic_{sample_idx:04d}.png')
                cv2.imwrite(save_path, synthetic_img)

        logging.info("合成数据生成完成")

    def _generate_piece_image(self, class_idx: int) -> np.ndarray:
        """
        生成单个棋子图像（简化示例）

        Args:
            class_idx: 类别索引

        Returns:
            生成的图像
        """
        # 创建64x64的空白图像
        img = np.ones((64, 64, 3), dtype=np.uint8) * 255

        if class_idx == 0:  # 空格
            return img

        # 根据不同棋子类型绘制不同的形状和颜色
        color = (255, 0, 0) if class_idx <= 7 else (0, 0, 255)  # 红方/黑方

        # 简单的圆形表示棋子
        cv2.circle(img, (32, 32), 20, color, -1)

        # 添加噪声和变形以增加多样性
        noise = np.random.normal(0, 10, img.shape).astype(np.uint8)
        img = cv2.add(img, noise)

        return img


def train_model(data_dir: str, save_path: str, epochs: int = 50,
                batch_size: int = 32, learning_rate: float = 0.001):
    """
    训练CNN模型

    Args:
        data_dir: 训练数据目录
        save_path: 模型保存路径
        epochs: 训练轮数
        batch_size: 批次大小
        learning_rate: 学习率
    """
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")

    # 数据变换
    train_transform = transforms.Compose([
        transforms.Resize((64, 64)),
        transforms.Grayscale(),
        transforms.RandomRotation(10),
        transforms.RandomHorizontalFlip(0.5),
        transforms.ColorJitter(brightness=0.2, contrast=0.2),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.5], std=[0.5])
    ])

    val_transform = transforms.Compose([
        transforms.Resize((64, 64)),
        transforms.Grayscale(),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.5], std=[0.5])
    ])

    # 数据加载器
    from torchvision.datasets import ImageFolder
    from torch.utils.data import DataLoader

    train_dataset = ImageFolder(os.path.join(data_dir, 'train'), transform=train_transform)
    val_dataset = ImageFolder(os.path.join(data_dir, 'val'), transform=val_transform)

    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False)

    # 模型和优化器
    model = ChessPieceCNN(num_classes=15).to(device)
    criterion = nn.CrossEntropyLoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
    scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=20, gamma=0.1)

    # 训练循环
    best_val_acc = 0.0

    for epoch in range(epochs):
        # 训练
        model.train()
        running_loss = 0.0
        correct = 0
        total = 0

        for batch_idx, (inputs, targets) in enumerate(train_loader):
            inputs, targets = inputs.to(device), targets.to(device)

            optimizer.zero_grad()
            outputs = model(inputs)
            loss = criterion(outputs, targets)
            loss.backward()
            optimizer.step()

            running_loss += loss.item()
            _, predicted = outputs.max(1)
            total += targets.size(0)
            correct += predicted.eq(targets).sum().item()

        train_acc = 100. * correct / total

        # 验证
        model.eval()
        val_loss = 0.0
        val_correct = 0
        val_total = 0

        with torch.no_grad():
            for inputs, targets in val_loader:
                inputs, targets = inputs.to(device), targets.to(device)
                outputs = model(inputs)
                loss = criterion(outputs, targets)

                val_loss += loss.item()
                _, predicted = outputs.max(1)
                val_total += targets.size(0)
                val_correct += predicted.eq(targets).sum().item()

        val_acc = 100. * val_correct / val_total

        print(f'Epoch {epoch+1}/{epochs}:')
        print(f'  Train Loss: {running_loss/len(train_loader):.4f}, Train Acc: {train_acc:.2f}%')
        print(f'  Val Loss: {val_loss/len(val_loader):.4f}, Val Acc: {val_acc:.2f}%')

        # 保存最佳模型
        if val_acc > best_val_acc:
            best_val_acc = val_acc
            engine = PieceRecognitionEngine(device=str(device))
            engine.model = model
            engine.save_model(save_path, epoch, val_loss/len(val_loader))
            print(f'  New best model saved! Val Acc: {val_acc:.2f}%')

        scheduler.step()

    print(f'Training completed! Best validation accuracy: {best_val_acc:.2f}%')