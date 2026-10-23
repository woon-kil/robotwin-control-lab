"""Core가 입력을 조용히 고치지 않을 때 쓰는 오류."""


class KinematicsError(Exception):
    """FK 또는 모델 구성이 계산 대신 실패로 끝났을 때."""

    def __init__(self, code: str, message: str) -> None:
        self.code = code
        super().__init__(message)
