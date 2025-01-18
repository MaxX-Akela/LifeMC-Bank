import disnake
from disnake.ext import commands
from disnake import Option
import json
import os
from datetime import datetime, timedelta
from disnake.ext.commands import has_role

class BankSystem(commands.Cog):
    def __init__(self, bot):
        self.bot = bot
        self.filename = 'bank_data.json'
        self.bank_data = self.load_bank_data()
        self.interest_rate = 5  # Процентная ставка по умолчанию (в % годовых)

    def load_bank_data(self):
        if os.path.exists(self.filename):
            try:
                with open(self.filename, 'r') as f:
                    return json.load(f)
            except json.JSONDecodeError:
                print("Ошибка чтения файла. Загружаем пустой словарь.")
                return {}
        return {}

    def save_bank_data(self):
        try:
            with open(self.filename, 'w') as f:
                json.dump(self.bank_data, f, indent=4)
        except IOError as e:
            print(f"Ошибка при сохранении файла: {e}")

    def calculate_interest(self, user_id):
        user_data = self.bank_data.get(user_id, {})
        deposit = user_data.get("deposit", 0)
        last_update = user_data.get("last_update")
        
        if deposit > 0 and last_update:
            last_update_date = datetime.fromisoformat(last_update)
            days_elapsed = (datetime.now() - last_update_date).days
            if days_elapsed > 0:
                # Начисление процентов
                yearly_rate = self.interest_rate / 100
                daily_rate = yearly_rate / 365
                interest = deposit * daily_rate * days_elapsed
                self.bank_data[user_id]["deposit"] += int(interest)
                self.bank_data[user_id]["last_update"] = datetime.now().isoformat()
                self.save_bank_data()

    @commands.slash_command(description="Проверить баланс и вклад участника.")
    @has_role(1327671090738237512)
    async def balance(self, inter):
        user_id = str(inter.author.id)
        self.calculate_interest(user_id)
        user_data = self.bank_data.get(user_id, {"balance": 1000, "deposit": 0, "last_update": None})
        
        embed = disnake.Embed(
            title=f"Финансовая информация {inter.author.name}",
            color=disnake.Color.blue()
        )
        embed.add_field(name="Баланс:", value=f"{user_data['balance']} ₽", inline=False)
        embed.add_field(name="Вклад:", value=f"{user_data['deposit']} ₽", inline=False)
        embed.set_footer(text=f"Запросил: {inter.author.name}", icon_url=inter.author.avatar.url)
        await inter.send(embed=embed)

    @commands.slash_command(description="Положить деньги на вклад.")
    @has_role(1327671090738237512)
    async def deposit_money(self, inter, amount: int = Option(name="amount", description="Сумма вклада", required=True)):
        user_id = str(inter.author.id)
        self.calculate_interest(user_id)
        user_data = self.bank_data.setdefault(user_id, {"balance": 1000, "deposit": 0, "last_update": None})
        
        if amount > user_data["balance"]:
            await inter.send("Недостаточно средств на балансе для вклада.")
            return
        
        user_data["balance"] -= amount
        user_data["deposit"] += amount
        user_data["last_update"] = datetime.now().isoformat()
        self.save_bank_data()

        embed = disnake.Embed(
            title="Вклад успешно оформлен",
            description=f"Вы положили {amount} ₽ на вклад.",
            color=disnake.Color.green()
        )
        embed.add_field(name="Новый баланс:", value=f"{user_data['balance']} ₽", inline=False)
        embed.add_field(name="Сумма на вкладе:", value=f"{user_data['deposit']} ₽", inline=False)
        await inter.send(embed=embed)

    @commands.slash_command(description="Снять деньги с вклада.")
    @has_role(1327671090738237512)
    async def withdraw_deposit(self, inter):
        user_id = str(inter.author.id)
        self.calculate_interest(user_id)
        user_data = self.bank_data.get(user_id, {"balance": 1000, "deposit": 0, "last_update": None})
        
        deposit = user_data["deposit"]
        if deposit == 0:
            await inter.send("На вашем вкладе нет денег.")
            return
        
        user_data["balance"] += deposit
        user_data["deposit"] = 0
        user_data["last_update"] = None
        self.save_bank_data()

        embed = disnake.Embed(
            title="Средства сняты с вклада",
            description=f"Вы сняли {deposit} ₽ со вклада.",
            color=disnake.Color.red()
        )
        embed.add_field(name="Новый баланс:", value=f"{user_data['balance']} ₽", inline=False)
        await inter.send(embed=embed)

    @commands.slash_command(description="Установить процентную ставку для вкладов (только для администраторов).")
    @has_role(1327671090738237512)
    async def set_interest_rate(self, inter, rate: float = Option(name="rate", description="Процентная ставка (в % годовых)", required=True)):
        if rate < 0:
            await inter.send("Процентная ставка не может быть отрицательной.")
            return

        self.interest_rate = rate
        await inter.send(f"Процентная ставка успешно изменена на {rate}%.")
    
# Добавляем Cog в бота
def setup(bot):
    bot.add_cog(BankSystem(bot))

