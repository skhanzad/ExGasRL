from streamlit import title, write, graphviz_chart
import numpy as np

# Author: Sourena Khanzadeh
class Dashboard:
    def __init__(self):
        self.title = "Explainable AI Framework"
        self.description = "This is a dashboard for the explainable AI framework"
        self.author = "Sourena Khanzadeh"
        self.version = "0.1.0"
        self.date = "2025-07-08"

    def run(self):
        title(self.title)
        write(self.description)
        write(f"Author: {self.author}")
        write(f"Version: {self.version}")
        write(f"Date: {self.date}")
        self.plot_data()

    def plot_data(self):
        graphviz_chart("""
        digraph G {
            A -> B;
            B -> C;
            C -> D;
        }
        """)

def main():
    dashboard = Dashboard()
    dashboard.run()

if __name__ == "__main__":
    main()