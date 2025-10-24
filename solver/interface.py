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
    iVBORw0KGgoAAAANSUhEUgAAABwAAAAcCAMAAABF0y+mAAAAAXNSR0IB2cksfwAAAAlwSFlzAAALEwAACxMBAJqcGAAAAtlQTFRFAAAAm6Oouby9m6Oozc3KanqGJ0JYKUNZKURYkZuhkZqhIj1UOFFkNk9jNU5jKURZxcbGZniELEdcNU1hM0xgN05iJ0JXlJ2kUWZ1MkxgNk5hNE1hN09jLUdbfouVrrK1QllqJ0NYKENZLUdcL0pfK0ZbJ0NYXnF91tTS0tHOYnSAIT1UKENYhpGaoqmtWGt6SV5wZneDsLS3bX2IJD9WK0ZbhJCXfouULEZbIDxTX3F+z87N//fw//fw+fHq8Onk+fHq//ny//fvwsPDSmBwHjtSNU9inKWqVml4PlVnXnF+zMzKs7a5P1dpHDlRSmBwtrm8+PDq//jx9e7o9Ozm+PDq+vLs9+/p8+vm9u/p9u7ooaisNk9iHTtSTGFxu7+/tbm6U2d2QVdpLkhdMUtfL0peJEBVLkhcmqOn9Ozn8+zm9u7o9e3n9e3n8+zm+/Ls//fv39zYc4KMJkFXLEdbMUpfNExgNk9iL0leX3B+5N/b/fXt+vHr/PTt/fTt0M/NRFtsMkxhNU5hNE1gM0xganqG9e3n//fv+/Pt//fw2tjVSF5vM0xhNE1gNU5iMUtgKkVaIj9VPlVnsra5+/Pt9Ozm9Ozn+fHr//bu49/bhpKaK0ZbJEBWKkRaM01hNk5iK0VaOVBknaSqkpyiLUdbHztSXG58xMbG//Xu//bw9Ozn+/Ps/vXu9e7onqWrOFFkHTpRRlxtsre4lZ+lNU5h19XTw8TE3dnW2dbTa3yHJEBWKENYd4WP4d3Z//bu//bv8uvl5+Pe9u7o//nx/PPtvsDBU2d3HTpSM01gnqWqycnJx8jHU2d2HjpRL0pdlp6koqqtRFtsPFRnV2p4qa+ycYCLIj9VeIaR39zXpauuQVlqJUFXKkVaMEpfKENYJkJYU2d3xsfHVGd3NU5hbX2JcX+KKURaN09iJkFWmaGnoaitJUBVL0hd1NLQdYSNMktfNk5iNU5gpKquJHGi3QAAAPN0Uk5TACHf3eHa8Pjr1937////7+Pf//7//vva4f/9/v/91ODm+/z////33bXk3vfu19nd5+Xi4PDs0t/y9OHe9//y7/j/9dfc9eTV4e7d4dvm9+PZ9P//////////6tXo+OPk5OLr///x9+fb7v7///7///7j2/H1//7+/9nj/f//+ePu//7+/tjq///74u7//f//9vji1PH+/v/+6dbr9Pj/7/Lt3Nbs8+Pp+f/////y3Or13dPS3H14GGHc8PTc4f3/6eT3//fg4/fq2N/Z3fbq2djg5ebd4fXaZNji9vX/9/nh4On93eb///rT4Pfn3Nbz+urYRAZnIQAAAPlJREFUeJxjZIABxv8M6IARQ2ToSDIxMv7BJcnKCATfsUtyMYLBB6ySghDJl1glJSCSj7BIykOlgPQNNElNRmRwCkXSnBEN7ARLeqALMz6RhTOTGQ86gN3ILMrIwMj4TOKFNFh8Su68ZEbpaIiid0zwyGZ6DnNBy4S8b7yMjBeEEe5mfGEClvrO2MMEUbRST+saA5Pmtf8vxHWh+vIZ5yM5BWjnSsmXv6EWMT5n3MrI6MmwA8PJ5kDjT8FsOo0kYXCRUQMt+G6CxJcs2g0k5bEE/KO1IZMZ3zTKYI2Vp4xzJl1mZBTHKvmKcXFbbjajMFZJhncga/kQfABsyzhtQZ0aegAAAABJRU5ErkJggg==
    '''

IMAGEDATA_TARGET = '''
    iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAYAAADgdz34AAAAAXNSR0IArs4c6QAAAERlWElmTU0
    AKgAAAAgAAYdpAAQAAAABAAAAGgAAAAAAA6ABAAMAAAABAAEAAKACAAQAAAABAAAAGKADAAQAAA
    ABAAAAGAAAAADiNXWtAAABb0lEQVRIDd3UMU4DQQwF0ECkVHQ0CPpcgoKalgJqGgR34Ry5AiUHo
    KFGdCDlABSU8F8YS4ElG4SyTb70d2bsb3t2PLuj0bZjkhc8D2fhU/jWaM7GR/MvnCXqOfxYQxra
    P2Mc5W1YiR8yvw6n4V6jORtf6cSIXYtK/h7lZbjTE8FHQ6uQ2F54VUIBx025m/EmvAvnjeZsfEB
    bRVYel2bVmdsVHIX3YR3Dz5GPBsTwy/Fr490IAufq1e1OAraX8CI8aDRn46OhFVM9kauDWSwCNA
    8cQSXfX1i+P9iqCC2IFSNXB+4257R5nLO13cJpKCGaAx8NLYi1lqsDHxGnqwgaau1YoHbLZg581
    rQg1lquBeoW1Hrj43KB2sVhq/LYxpM2XmV8bTSH8pW2YivXl6o9Zxm93mBNHvyaTrL7QT80JzXo
    r0IBGPRnp8B4qYimb/x3rQg4ruqJQqtIs/IPGl8vNN7tcoV9/r5QNGfjo9lifALEXqkt2d8oHAA
    AAABJRU5ErkJggg==
    '''

IMAGEDATA_obstacle = '''
    iVBORw0KGgoAAAANSUhEUgAAABgAAAAYCAMAAADXqc3KAAAAAXNSR0IB2cksfwAAAAlwSFlzAAALEwAACxMBAJqcGAAAAYlQTFRFAAAA3trXmaCnlJ2kdIKNWm17VWh4b3+LkZuin6er2tjUrbK1coGMOlFlMEleMUpfL0leN1BkfYuUr7S2fImTM0xhMkxgNE1hNE1gMkxhNE1hbHyI1NLQPVVnMktgM0xgM0xgM0xhNk9it7q7p66xLkhdNU5iLkddiJObaHiFLkhdNE1hTmNzW258L0peNExgPVZnW218L0leOlJl5N/caHmGLkhdRFtrpKquLUdcNU1hLkhdeoeSuLu8PFRnMkxft7m8kpyiMEpeLUZba3uHoqqtd4aPMktgMktfR11uVmp4SmBwOFFkM01hMkxgi5WdycjHkJqgXnB+Nk9jcYCLrrK23tzYY3SCQlprP1ZpW25819bT5eHcpayuXXB9UWV1PFRmM0xgOlJlYnOAWGx6rLG03trXnKSpW258LkhdLUhdNE5iLUdcTGFxd4aRZHaBYXJ/TGFxLkleL0peOVFjpauvxcXFeYeQQlpqL0peNk9imKCmcH+KL0ldRlxtQVhqM0xgOFBkNk9iLZCXpQAAAIN0Uk5TAAUfk9fU2+SZJgdRru36//vwylS87//////up5fs///+//GT2/j//ODg//7h4P/+2+L/2RPf/trd+v/92Uru8ZPh7/rcR9/7/ulKSPL++sc8GX32pitHrPLvq4UGZI+98vzz3ZJ6NEfc7///8eFGCUnl/P7m3UNO6v3uqtz66un78fM4UHZaAAABE0lEQVR4nGNkgABGOPgGFYBKcIPEwNJvUSVEwOL/mRgZn6JIyIDEoSbeRZZQYfzEAWawMzLeQJbQZGT8AmbwPJG9giyhy/gXIiHwif8CsoQh488vUGcLMZ5BkjBlfAixg/Gf5ElkHRaMb3+CGdKMjEeQJWwZH0BYPzQYD6B40JHx3n+QuA7jHlSfuzIyXuAAO3sbqgSDN+OV/wx6QI9vQJUIZAS7Up+NcTWKRBgj4wmQHYyWjIxL4RIx4HjYB+Y7gdmzwBLpIOYaDqiZP/hZgJKTGBjzgaJbWL/+hzmCgZH7ty9IaTkj46YvDKjgBzvQ9BrGvU8Z0MGPH/mM9dM5McSBIJGxcR7jf2wyjE1zsQkzMAAAk8w+RiGRmbgAAAAASUVORK5CYII=
    '''
