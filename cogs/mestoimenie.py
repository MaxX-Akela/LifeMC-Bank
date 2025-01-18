from disnake.ext import commands

guildid = 1292191704320835585 # id сервера где будет местоимение
pronounstext = 'Разработано thetelin & Akela' # текст в местоимение

class PronounsBot(commands.Cog):
    def __init__(self, bot):
        self.bot = bot

    @commands.Cog.listener()
    async def on_ready(self):
        await self.bot.http.edit_my_member(guildid, pronouns=pronounstext)
        print("Местоимение установлено!")

def setup(bot):
    bot.add_cog(PronounsBot(bot))
