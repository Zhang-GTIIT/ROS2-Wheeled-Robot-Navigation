import os
import sys
import ssl
import smtplib
from email.message import EmailMessage

def send_email(subject: str, body: str):
    # 从环境变量读取配置
    SMTP_SERVER = os.environ.get("SMTP_SERVER", "smtp.qq.com")
    SMTP_PORT = int(os.environ.get("SMTP_PORT", "587"))
    SMTP_USER = os.environ.get("SMTP_USER")
    SMTP_PASSWORD = os.environ.get("SMTP_PASS")
    EMAIL_FROM = os.environ.get("EMAIL_FROM", SMTP_USER)
    EMAIL_TO = os.environ.get("EMAIL_TO")

    missing = [
        name
        for name, value in {
            "SMTP_USER": SMTP_USER,
            "SMTP_PASS": SMTP_PASSWORD,
            "EMAIL_TO": EMAIL_TO,
        }.items()
        if not value
    ]
    if missing:
        print(
            f"❌ 缺少邮件环境变量: {', '.join(missing)}。请参考 .env.example。",
            file=sys.stderr,
        )
        return False
    
    print(f"配置信息:")
    print(f"  SMTP服务器: {SMTP_SERVER}:{SMTP_PORT}")
    print(f"  发件人: {EMAIL_FROM}")
    print(f"  收件人: {EMAIL_TO}")
    print(f"  用户名: {SMTP_USER}")
    print(f"  密码: {'已设置' if SMTP_PASSWORD else '未设置'}")

    # 创建邮件
    msg = EmailMessage()
    msg["Subject"] = subject
    msg["From"] = EMAIL_FROM
    msg["To"] = EMAIL_TO
    msg.set_content(body)

    try:
        context = ssl.create_default_context()
        
        # 针对不同端口使用不同的连接方式
        if SMTP_PORT == 465:
            # SSL 连接
            with smtplib.SMTP_SSL(SMTP_SERVER, SMTP_PORT, context=context, timeout=30) as server:
                server.login(SMTP_USER, SMTP_PASSWORD)
                server.send_message(msg)
        else:
            # STARTTLS 连接（端口 587 等）
            with smtplib.SMTP(SMTP_SERVER, SMTP_PORT, timeout=30) as server:
                server.ehlo()
                if SMTP_PORT == 587:
                    server.starttls(context=context)
                    server.ehlo()
                server.login(SMTP_USER, SMTP_PASSWORD)
                server.send_message(msg)
        
        print(f"✅ 邮件已成功发送至 {EMAIL_TO}")
        return True
        
    except smtplib.SMTPResponseException as e:
        # 检查是否是我们知道的无害错误
        if e.smtp_code == -1 and e.smtp_error == b'\x00\x00\x00':
            print(f"⚠️  邮件发送成功，但退出时出现无害错误（可忽略）")
            print(f"✅ 邮件已成功发送至 {EMAIL_TO}")
            return True
        else:
            print(f"❌ SMTP 响应异常: {e}", file=sys.stderr)
            return False
    except smtplib.SMTPAuthenticationError as e:
        print(f"❌ SMTP 认证失败: {e}", file=sys.stderr)
        print("请检查用户名和密码是否正确")
        return False
    except smtplib.SMTPException as e:
        print(f"❌ SMTP 错误: {e}", file=sys.stderr)
        return False
    except Exception as e:
        print(f"❌ 发送邮件时出现错误: {e}", file=sys.stderr)
        return False

if __name__ == "__main__":
    # 如果传入命令行参数，则用参数作为正文
    if len(sys.argv) > 1:
        SUBJECT = sys.argv[1] if len(sys.argv) > 1 else "【通知】来自 Python 脚本的消息"
        BODY = sys.argv[2] if len(sys.argv) > 2 else " ".join(sys.argv[1:])
    else:
        SUBJECT = "【通知】来自 Python 脚本的消息"
        BODY = "这是一条由简化脚本自动发送的测试消息。\n\n你可以在这里写任何你想发送的内容。"
    
    print(f"发送邮件:")
    print(f"  主题: {SUBJECT}")
    print(f"  正文: {BODY[:50]}..." if len(BODY) > 50 else f"  正文: {BODY}")
    
    success = send_email(SUBJECT, BODY)
    
    if success:
        sys.exit(0)  # 成功退出码
    else:
        sys.exit(1)  # 失败退出码
