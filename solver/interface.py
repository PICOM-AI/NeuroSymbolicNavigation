import tkinter as tk


class Interface(tk.Frame):

    def __init__(self, instance):
        self.frame = tk.Tk()
        self.instance = instance

        # size for each cell in pixel
        self.size = 32

        # colors for the board
        self.color1 = "white"
        self.color2 = "grey"
        self.pieces = {}

        canvas_width = self.instance.size * self.size
        canvas_height = self.instance.size * self.size

        tk.Frame.__init__(self, self.frame)
        self.canvas = tk.Canvas(self, borderwidth=0, highlightthickness=0,
                                width=canvas_width, height=canvas_height,
                                background="bisque")
        self.canvas.pack(side="top", fill="both", expand=True, padx=2, pady=2)
        # this binding will cause a refresh if the user interactively
        # changes the window size
        self.canvas.bind("<Configure>", self.draw)

        self.pack(side="top", fill="both", expand=True, padx=4, pady=4)
        self.robot_image = tk.PhotoImage(data=IMAGEDATA_robot)
        self.target_image = tk.PhotoImage(data=IMAGEDATA_TARGET)
        self.restricted_area_image = tk.PhotoImage(
            data=IMAGEDATA_restricted_area)
        self.obstacle_image = tk.PhotoImage(data=IMAGEDATA_obstacle)

        self.add_piece("robot", self.robot_image, self.instance.robot)
        self.add_piece("target", self.target_image, self.instance.target)
        for count, restricted_area in enumerate(self.instance.restricted_areas):
            self.add_piece("restricted_area_" + str(count),
                           self.restricted_area_image, restricted_area)
        if hasattr(self.instance, "obstacles"):
            for count, obstacle in enumerate(self.instance.obstacles):
                self.add_piece("obstacle_" + str(count),
                               self.obstacle_image, obstacle)

    def add_piece(self, name, image, position):
        '''Add a piece to the playing board'''
        self.canvas.create_image(0, 0, image=image, tags=(name, "piece"),
                                 anchor="c")
        self.place_piece(name, position)

    def place_piece(self, name, position):
        '''Place a piece at the given row/column'''
        self.pieces[name] = position
        x0 = ((position[0] - 1) * self.size) + int(self.size / 2)
        y0 = ((position[1] - 1) * self.size) + int(self.size / 2)
        self.canvas.coords(name, x0, y0)

    def remove_pieces(self, name):
        remove_keys = []
        for key in self.pieces.keys():
            if key.startswith(name):
                remove_keys.append(key)
        for key in remove_keys:
            self.pieces.pop(key)
            self.canvas.delete(key)

    def draw(self, event):
        '''Redraw the board, possibly in response to window being resized'''
        xsize = int(event.width / self.instance.size)
        ysize = int(event.height / self.instance.size)
        self.size = min(xsize, ysize)
        self.canvas.delete("square")
        for row in range(self.instance.size):
            for col in range(self.instance.size):
                color = self.color1 if (col + 1,
                                        row + 1) not in self.instance.walls else \
                    self.color2
                x1 = (col * self.size)
                y1 = (row * self.size)
                x2 = x1 + self.size
                y2 = y1 + self.size
                self.canvas.create_rectangle(x1, y1, x2, y2, outline="black",
                                             fill=color, tags="square")
        for name in self.pieces:
            self.place_piece(name, self.pieces[name])
        self.canvas.tag_raise("piece")
        self.canvas.tag_lower("square")


IMAGEDATA_robot = '''
    iVBORw0KGgoAAAANSUhEUgAAABwAAAAcCAYAAAByDd+UAAAAAXNSR0IB2cksfwAAAAlwSFlzAAALEwAACxMBAJqcGAAABQdJREFUeJzdVVtPG0cU9h/oXyjh4gu3KoikqRKUVq1UhZcoeUj7kkitmqqq+lD1ojZ96ENVtZWiPgRfIJQ7BAgKoS40CW0o7NprwHjxZdcYxzZ4sQ1eG9u7vmBswJ7O2GuHEJI2VH7pSJ9mL+fMd853zsyISNLAiF5gxFanHWH9z1TUOWl4Eb8jjchiG832HY8GemrTbG99iCMVlpKRxdnll9j+xo1wrxSwPXUg3CsD/r7jfHxN/7AkhDHvwp+QKBHsqc36e2ozgd76zEZH+Q5v/229JIRoBEbeYMI9YuDvqQdcvwywt0464xt0Y8kI+UcTVGDotQ2+uzIdGmxk+OUxc8nICsPjnNV8pvxmYlwz3lVyssJ4e4ieumNewUtG4PX7al0+73Wa8Vgtq4xJbV6e0jlc87SbsVrcDG1lGJpZ95w90uJ6p9v+/V+U8cLQPNM8MOt+vVsXaurW8ac6iUjDTU0IInyqQ8ufaNcW3kMN7drNk50E29Spdb3ZO7t67paeuzy6YJmyrUw9l+zL+0ZarMRilXIsW6HQAIQqpRZCAyRwlqoICC0Qw2eJqvCeh6SVAGJV3hb5VSLI8dBXD0zYoWSfjC9aj7VgmWroWK3CQY0SzyJU/2tg0B4r+uT823Sg7AaWvvbArH+C7I7RQVe0YNs1rVpQXEDIoFr4JlPi4FnI22ty9jI4Vxf/wQDaCJgpFhunHGSR8NKI3lWhJHJZIUMkV5ViJnniFy0rUeBxJFP1s8lyElfIsRSsaxCqs73fHgVTCQO5ODTvypEZV9wr9a14VKZCmWBZRHami/AOGuyk0+dbukc5l84PzXlRXQ+S5sk04K2+We8wabfaPJ7lSdrpvji8EKpS4I+DhGvWtWoSBpfbKho1OkxSBbaDJEE/XmnTRHH7igkkQ8lMPAxAYjPr2/DRZ7sJrxhJLKiAZtRADTe1Ebg9LGArlN4T7OHWcUN1eKlKKAeSG3IodUuk6Idpyghl26uBjYK6C8q7vhsNhFPREEiE/JlEJABALJD6bspiK4dNVYMCE7JDnfjeGGlP8yyH7OPQfju6CXZ4NnhVvegqV2iB0ESgSo5nr/5Krog+VJM2sQIDOUI5Dj5SkwwkTG5xgWwy4odJstldjs206mzWypaZvBICIbL/etJkg4SxJBfMbkXYDJpBjI18cc/kLJNr0LrZQnCwjmu5hhEr8tsA7bnmgTnfNscy2S0OQGewl+AAiAcjH4/DiCEBCqxAiOzPD877khE2AKA9UiMD5xTn9747ondUKgmhg3HUqZBwbkl0ZXTBKRYKjKKHz+kfp2lLimc9mViQg3L61JRjERY9imoi279FhNq0zy3Tad7vzcQCUZAIssOLdhKSJGQH5L8wOL8u+vahxSRRYJnCIlJ0qiiw3Xdu650/TVPkB2MGHcyKkwrOTyNHun3ptp66jlHmz+8bTXUqnBeCe9zNkOPT341WkdrssEiVWFq2b0GUKZILSYhm2T7nQyHYH2vJN95T9ioNqmNq3OI0itbWve+f7iI20Fm4/0TJHU8q/B9PmYP2Nfn3ok+uN1Q6cK5/1l08aVTEEvUyPPNqBNL/AFAEIoeZS1p1SM7AH1aX8Ynz9NqkyVIux5L52+EoEG6IHPJdWXZjJvNqB7GmNj+aO/TGmKAc1JW7BuvpTiLc1EWEz7wAmvtnvdDXfvnuwjLcQlZ4p5q69TaLy+sZeO6d+L8bfwMUiC8mf5hc3gAAAABJRU5ErkJggg==
    '''

IMAGEDATA_restricted_area = '''
    iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAMAAADXqc3KAAAAAXNSR0IB2cksfwAAAAlwSFlzAAALEwAACxMBAJqcGAAAAfhQTFRFAAAALSwsHx8fHx4eHh4eHx8eRz4c98wS9soR9ssQ98wR9ssRHh0dIB8fKSkoIiIiFBQUFRQUGhkUcmAO+MoF98kF9McFtJUKNS8TaFocISEhiXMN68AG98oF9soFm4ELIB4UKCQTwp8J98wTIiEhh3ENm4EMLCcTMCsStpYJ9ssTFBMUGBcUeWYN8MQG9cgFs5QKGxoUvp0J98wSISAgHRsUln0L98sF78QGmH4LNzASy6cIMy8gdmMO9MgFmoAMJiMTMy0SsZMK9skFpowY8cUFHh0UOTISsJEK9swT98wU98oE+MsFnoMLJCETyqYIr5EKX1Mc9ssS88cFl34LKCUTLyoSuZkKxqMILSgSIB8f9ssR88YFuJgJHBoUtJQK9sgFxaIIJiITICAfl34MMiwSv50IspMKKiYTIB8goocLJyMTqIsKuZkJNC4SHx8fJSITLyoTtZYKt5cJICAgS0MfIB4Tx6QIrI4KLikSu5oJw6EJQzweIiIisJEJvZwJIR4UHBsUvJwJ98kEqo0KIyATlXwL9csTKycTrY8KuJgKFxcUknoMICAgHBsTv50JJyQTo4cL8cYFMSwSx6QJpooLoYcL98wUKikpl4AS8scM9soM9ssM9soL98oMNjEaHBwbHBscGxsbGxsbaVkW7cQM98sM9s0ZpxdyggAAAKh0Uk5TACZJSUlJSUlJSUlJSUklg////////////4SD//////////+Eg///////hP//////////hIL/////////g/////////+C/////4SD////////hIL/////////g4H/////////hP//////hP//////g/////+Dg////////4SE////////////hP//////gv///////////4Rivr2+vr29v76/v76+vr1i7wMofQAAAcBJREFUeJx1kk8o5AEUx79fsn7THNiRyf6m/N2m1mF2xBoH7Uk2Dg5kIklbbMnBSZEocbS126phta0UOa1ITdFetOwa2WWUGGkvi5SGtvjNHDTe+zl7l9/vfb7vvd/r+3vEI0FQIpVGO+7SaSlz8D+RJRwqpLQmaZcbPCfMW3lzUlpTlpPXyp+SJ8TzS6TlWEkYSGbzQnkeL3MPpEN0XiUMuHht1xfxLu7eISrIc/MM8PAwpbyURw48kVHV5F8FxdzVRxkZgy3U8FDXNgojygPcMQDTFSbq9+Hjnp/r9vz8TeEo5yLRuA1UkmvKa7mZqNwqNcl5om3d+4xcVu4v4HdvTIZ/a/qio4q8v/4pb+YJYt5oIxfQGhIhEuSc51TmuBfgQKKFs0DHRxHSS2a0/q2AQNTq+P1Hks5xom/b3v8dv2ZagcQrhjTTUbaJaHH9OBB3u+avNHvtGyOGPokdDRkh2d/q4QfBnuD+Tfiho+7FVtVni73RVa3v40b1gP4o9HNCyl++Cet3McifEfXq/egwV8Q3tLunT1UY4VJMBXOA03HJ+4/trf3BqTis0W5iUq8AD8dAsZm3zhsnW/nY+dwDmQKGGWgLvCMAAAAASUVORK5CYII=
    '''

IMAGEDATA_TARGET = '''
    iVBORw0KGgoAAAANSUhEUgAAABwAAAAcCAMAAABF0y+mAAAAAXNSR0IB2cksfwAAAAlwSFlzAAALEwAACxMBAJqcGAAAAltQTFRFAAAA4G5r31tW31pW31lV31pV3llV32hl3l5a3j043j453z453j443lFO3khD3j853kA73jo13js13zs23js23kA73j863kQ/30dD30A73zs1ti8qRBIQRBIRRRIRRRMRwDMu3j863khD3V1Z3kM+3j85ry4paBsXAgICcR4brS0q3z8630dD3mNf31hU3z043To0rS0pPxEPEQYGTxYUFQcHPBEQqywo2zo13z4531tX4HNy3lxZ3z45yzQvShMRCgMDPA8O3jk1RhMRCAMDRRIQyDMv3j4531tX32ln3lJO3z03HgkIDwYFcR0b2jo0yjUwyjQw2DgzXRkXEAYGEwUF3lFN3lJN3zw3eSAd0jcytDArKgwLJQoJqCso3zo10DcygyIg3lJN3lFNhyMgvjEtLgwLBwMDBgMDKgsKtzAskiYi3lJO31NP3z04hiMgsy8rqy4qkiUi31JO3z03zDUwZhsZDgUFCgQEXhgWyTUw30tG3zs1RRMSoisnyDUweyAdeB8cwDItlyglShUT31RQ3kZC3z04axwZIgkJQBEQnSsnoSwoMw4NIAkIZBkW3j043kdC3lxX31pV3z442jk0ahwZFAYGJgsLtjArtS8rLw4NEwYGZhsY2Dkz3j4531pW315Z3kM+3jw23Do0aBsYDgUEAwICBQICDwUFYxwZ30M+4FlV3mNf3kxH3j86iyMglick3kA730tI4VlW31NO3kVA3jo01DYyuC8ruDAr1zgz30I831RP30lF3j443kZB3kM+3j4530M+32di3kZB3kdB30ZB3llWTEScbQAAAMl0Uk5TAAYWFhYWFgVD4uLi4jZdv9L/////0L9optz/////////368OUu////////dWEibf////////////5DEGPuP////////////nRwY4//////////////9AOP////////////9AOP//////////QDj//////zj+////////Nfz//////////yO09P//////////9rkqJt//////////////5DAl2f//////////3i0HKur///UqCC2X//////+jMDRplo5qOSR5eXkdbDMA6AAAAVJJREFUeJxjZMADGGkuyQgG/5l+YpPkAMn9Y2D+jCnJB5JiAGoEUi/RJCVghv5n+sLLeA9ZUpmR8SsPIyPU6g+CjIyXEZJ6IF3fuEEcxs9gC47BJa1B3JcSjBcMzzMYMT6RBXG3wyS9GM8b3VFlPGoDFNzvxHhFd+9/11UwyXDG3ZZ3DBm3e0FcdVx337/AeTDJ5BdAnTa/2cBSX3n26u5liJoCk8xlXGN0x/2D4A4g2/Od8Dbd/f9c4MYWMy4D6RQD63wKlNz3D2FnNeN0yzuhjHNSgXL9RYxLdPf68sKNbQXpmJ/E2P+d83s14+w0EDcPHgiTQdyJBcCwZWDsLQFxEhAhtBDiBxAHGPJtT6YzhiAk1zIyJtknVIDZnSefBjMy+iDHylaozv+MjH9YGZ3R4nMfI2PgshUMm/ySGBltMVPCEWB0MgDj0wIzJWADNJIEAHR5YB1h0WcnAAAAAElFTkSuQmCC

    '''

IMAGEDATA_obstacle2 = '''
    iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAMAAADXqc3KAAAAAXNSR0IB2cksfwAAAAlwSFlzAAALEwAACxMBAJqcGAAAAYlQTFRFAAAA3trXmaCnlJ2kdIKNWm17VWh4b3+LkZuin6er2tjUrbK1coGMOlFlMEleMUpfL0leN1BkfYuUr7S2fImTM0xhMkxgNE1hNE1gMkxhNE1hbHyI1NLQPVVnMktgM0xgM0xgM0xhNk9it7q7p66xLkhdNU5iLkddiJObaHiFLkhdNE1hTmNzW258L0peNExgPVZnW218L0leOlJl5N/caHmGLkhdRFtrpKquLUdcNU1hLkhdeoeSuLu8PFRnMkxft7m8kpyiMEpeLUZba3uHoqqtd4aPMktgMktfR11uVmp4SmBwOFFkM01hMkxgi5WdycjHkJqgXnB+Nk9jcYCLrrK23tzYY3SCQlprP1ZpW25819bT5eHcpayuXXB9UWV1PFRmM0xgOlJlYnOAWGx6rLG03trXnKSpW258LkhdLUhdNE5iLUdcTGFxd4aRZHaBYXJ/TGFxLkleL0peOVFjpauvxcXFeYeQQlpqL0peNk9imKCmcH+KL0ldRlxtQVhqM0xgOFBkNk9iLZCXpQAAAIN0Uk5TAAUfk9fU2+SZJgdRru36//vwylS87//////up5fs///+//GT2/j//ODg//7h4P/+2+L/2RPf/trd+v/92Uru8ZPh7/rcR9/7/ulKSPL++sc8GX32pitHrPLvq4UGZI+98vzz3ZJ6NEfc7///8eFGCUnl/P7m3UNO6v3uqtz66un78fM4UHZaAAABE0lEQVR4nGNkgABGOPgGFYBKcIPEwNJvUSVEwOL/mRgZn6JIyIDEoSbeRZZQYfzEAWawMzLeQJbQZGT8AmbwPJG9giyhy/gXIiHwif8CsoQh488vUGcLMZ5BkjBlfAixg/Gf5ElkHRaMb3+CGdKMjEeQJWwZH0BYPzQYD6B40JHx3n+QuA7jHlSfuzIyXuAAO3sbqgSDN+OV/wx6QI9vQJUIZAS7Up+NcTWKRBgj4wmQHYyWjIxL4RIx4HjYB+Y7gdmzwBLpIOYaDqiZP/hZgJKTGBjzgaJbWL/+hzmCgZH7ty9IaTkj46YvDKjgBzvQ9BrGvU8Z0MGPH/mM9dM5McSBIJGxcR7jf2wyjE1zsQkzMAAAk8w+RiGRmbgAAAAASUVORK5CYII=
    '''
IMAGEDATA_obstacle = '''
    iVBORw0KGgoAAAANSUhEUgAAABoAAAAaCAMAAACelLz8AAAAAXNSR0IB2cksfwAAAAlwSFlzAAALEwAACxMBAJqcGAAAAeNQTFRFAAAA/f78FRYWFhYWExITEhISFRUVGRkaFBQUExQUOTk5VFRUOzs8FhYXGhobFBMTEhESkZGS7OzsmpqaEhISExMTExMTERER7OvsmZmZERERHBscGBgYUVFRfX19VVVVFRUVGhoaHh4eEhITGxsbFRUVW1pbFRUVExITExMTEhITERESFBQUExMTFhYWSkpKHBwcExMTFxcYGBgYGBgYGRkZFxcXFhYWFxYXExMUGxsbOzs7REREFxcXUVFRSUhJTk1OUE9QRkZFPDw8Pj4+Pz4/FBMUFhYXNjY2Pj49IiIiICAgGhoaIyMjExMTFhcXOjk6Pz8/HR0dGhoaFBQVHx4eExITFhYWRkZGHBwcFhYWFhYWICAgEhITOTg5QUBBGBgYGxsbHx8fExITFhYWOjo6R0dHGBgZExMTHBwcFxYXHh4eFBQUGBgYPT09FxcXm5uaHx8fICAgfH18GhoaHx8fGxsbhYWFFBQVFhYXgIGAHR0dGRgZhISEFBQUFhUWcHBwHBwcmZmZFRUVc3NzGBgYISAhGhobkJCRFhYWenp6GxobmZiZFxcXgICAFxcXJSUlHh4eICAhNTU2FhYWFxcYOjo6HBwcIyMjJiYmISEhIyMjHx8fHx8gJCQkISAhLy8v6R8aCQAAAKF0Uk5TAAAWqfn5rxcgrP///60Va///////TGv///9LNsb////FJjK5yDIMs9DQ6//vsAUJt/69vb6+vby8+rsLCbwKCgoKCQkJCfHBDAlBT0ow8cEMCWl+d03xwQlpfHhO8AwJfXpO8MIMBWOFaHhNfmUGfAOqrAZ5TWgD2N0Ie30D2NwHewTZB3hOaATcCGcE3Qh3UGmfQeLmRZhPT7y2wMG3uzyl4rA2AAAA8ElEQVR4nGNkwAkYUdj/cUgxMYLATyxSHEBxBiD+hCnFD5V6jSklxviLkY3xN9szTClpqK77WJyhxPiRQYDxNoYz1MDOeynxh5GF8RKKlD4jDIDMPIosZcN4WQ8uy7gDWcrz3V0zxq1Ahg9IagOyVOCzB9aMa4GMEKDMRxRdEfceOjEuAzKiGRl/sS9Cloq/8UWPfQGQkQgycDayVBpQ4DtIKgskNQnNy5lsEOd1Y4ZG2RNZkMzzeVikILq+YNEVxKfE+IeVkbEKU6r9K88HQcbfd7EY2H3C8jsX0MQCTKmJ0ADMwJSaCZVKwJTCAHikAF4lNxtlbX0JAAAAAElFTkSuQmCC
    '''