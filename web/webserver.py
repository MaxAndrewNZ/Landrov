import tornado.ioloop
import tornado.web


#main_page='''Content-Type: text/html; charset=utf-8
main_page='''<html>
<H3>Welcom to land rov</H3>
<a href="http://192.168.8.106:8080/static/web_joy.html">Web Joystick!</a>
</html>
'''
class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.write(main_page)

def make_app():
    return tornado.web.Application([
        (r"/", MainHandler),
         (r'/static/(.*)', tornado.web.StaticFileHandler, {'path': '.'}),
    ])

if __name__ == "__main__":
    app = make_app()
    app.listen(8080)
    tornado.ioloop.IOLoop.current().start()
