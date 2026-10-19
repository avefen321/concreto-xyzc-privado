"""Private ASGI entrypoint; the local server and geometry engine are unchanged."""
import base64
import binascii
import hashlib
import os
import secrets

from starlette.responses import JSONResponse, PlainTextResponse


class PrivateWebApplication:
    def __init__(self, application, username, password):
        self.application = application
        self.username_hash = hashlib.sha256(username.encode('utf-8')).digest()
        self.password_hash = hashlib.sha256(password.encode('utf-8')).digest()

    def authorized(self, scope):
        headers = dict(scope.get('headers', []))
        header = headers.get(b'authorization', b'')
        username = password = ''
        try:
            scheme, token = header.split(b' ', 1)
            if scheme.lower() == b'basic' and len(token) <= 8192:
                decoded = base64.b64decode(token, validate=True).decode('utf-8')
                username, password = decoded.split(':', 1)
        except (ValueError, UnicodeError, binascii.Error):
            pass
        # Hash first so compare_digest always receives equal-sized byte strings.
        user_ok = secrets.compare_digest(hashlib.sha256(username.encode('utf-8')).digest(), self.username_hash)
        password_ok = secrets.compare_digest(hashlib.sha256(password.encode('utf-8')).digest(), self.password_hash)
        return user_ok and password_ok

    async def __call__(self, scope, receive, send):
        if scope['type'] == 'lifespan':
            return await self.application(scope, receive, send)
        if scope['type'] == 'websocket':
            return await send({'type': 'websocket.close', 'code': 1008})

        async def private_send(message):
            if message['type'] == 'http.response.start':
                replaced = {b'cache-control', b'x-robots-tag', b'referrer-policy'}
                headers = [(name, value) for name, value in message.get('headers', [])
                           if name.lower() not in replaced]
                headers.extend([(b'cache-control', b'no-store'),
                                (b'x-robots-tag', b'noindex, nofollow, noarchive'),
                                (b'referrer-policy', b'no-referrer')])
                message = dict(message, headers=headers)
            await send(message)

        # A minimal liveness response is the only unauthenticated HTTP resource.
        if scope['path'] == '/healthz' and scope['method'] in ('GET', 'HEAD'):
            response = JSONResponse({'status': 'ok'})
        elif not self.authorized(scope):
            response = PlainTextResponse('Authentication required', status_code=401,
                headers={'WWW-Authenticate': 'Basic realm="XYZC privado", charset="UTF-8"'})
        else:
            return await self.application(scope, receive, private_send)
        return await response(scope, receive, private_send)


def create_app():
    username = os.environ.get('XYZC_PRIVATE_USER', '')
    password = os.environ.get('XYZC_PRIVATE_PASSWORD', '')
    if not username or ':' in username or any(ord(c) < 32 for c in username) or len(password) < 24:
        raise RuntimeError('Configure XYZC_PRIVATE_USER y XYZC_PRIVATE_PASSWORD (mínimo 24 caracteres); no se permite iniciar sin protección.')
    from .main import app
    return PrivateWebApplication(app, username, password)


def main():
    import uvicorn
    # The converter keeps its last eight results in process memory: one worker.
    uvicorn.run('backend.hosted:create_app', factory=True, host='0.0.0.0',
                port=int(os.environ.get('PORT', '8000')), workers=1)


if __name__ == '__main__':
    main()
