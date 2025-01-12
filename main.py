import disnake
from disnake.ext import commands
import os

intents = disnake.Intents.default()
intents.members = True

bot = commands.Bot(command_prefix="!>", intents=intents)

@bot.event
async def on_ready():
    print(f"Мы вошли как {bot.user}")
    await bot.change_presence(status=disnake.Status.dnd, activity=disnake.Game("LifeMC | 1 Сезон"))

bot.load_extensions("cogs")

bot.run("")
